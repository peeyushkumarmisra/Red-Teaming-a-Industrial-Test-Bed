import datetime
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ids_security import check_keystore, make_keystore  # noqa: E402
from ids_security.checks import check_enclave, find_enclaves, signed_content  # noqa: E402
from ids_security.keystore import (build_keystore, enclave_dir, load_cert,  # noqa: E402
                                   signed_by)
from ids_security.policy import default_policy_path, load_policy  # noqa: E402

OPENSSL = shutil.which('openssl') is not None


@pytest.fixture(scope='module')
def policy():
    return load_policy(default_policy_path())


@pytest.fixture(scope='module')
def keystore(tmp_path_factory, policy):
    out = str(tmp_path_factory.mktemp('ks'))
    manifest = build_keystore(policy, out)
    return out, manifest


def failures(findings):
    return [f.text for f in findings if not f.ok]


def test_legitimate_enclaves_pass_the_checker(keystore):
    out, _ = keystore
    assert find_enclaves(out) == ['/cell/plc', '/cell/robot', '/monitor/ids']
    for enclave, domain in (('/cell/robot', 10), ('/cell/robot', 20), ('/cell/plc', 10),
                            ('/cell/plc', 20), ('/monitor/ids', 20)):
        assert failures(check_enclave(out, enclave, domain)) == []


def test_the_ids_enclave_is_not_granted_the_cell_domain(keystore):
    out, _ = keystore
    bad = failures(check_enclave(out, '/monitor/ids', 10))
    assert len(bad) == 1 and 'permissions cover domain 10' in bad[0]


def test_loot_holds_a_stolen_and_a_forged_identity(keystore):
    out, manifest = keystore
    loot = os.path.join(out, 'loot')
    ca = load_cert(os.path.join(out, 'public', 'ca.cert.pem'))
    implant = load_cert(os.path.join(enclave_dir(loot, '/attacker/implant'), 'cert.pem'))
    forged = load_cert(os.path.join(enclave_dir(loot, '/attacker/forged'), 'cert.pem'))
    assert signed_by(implant, ca)
    assert not signed_by(forged, ca)
    assert failures(check_enclave(loot, '/attacker/implant', 10)) == []
    assert failures(check_enclave(loot, '/attacker/forged', 10)) == []
    assert not os.path.exists(os.path.join(loot, 'private'))
    assert {e['path']: e['ca'] for e in manifest['enclaves']}['/attacker/forged'] == 'forged'


def test_legitimate_containers_never_see_the_private_ca(keystore):
    out, _ = keystore
    for enclave in ('/cell/robot', '/cell/plc', '/monitor/ids'):
        files = sorted(os.listdir(enclave_dir(out, enclave)))
        assert files == sorted(['cert.pem', 'key.pem', 'identity_ca.cert.pem',
                                'permissions_ca.cert.pem', 'governance.p7s', 'permissions.xml',
                                'permissions.p7s'])


def test_signed_permissions_match_the_xml_and_order_the_rules_for_cyclone(keystore):
    out, _ = keystore
    d = enclave_dir(out, '/cell/robot')
    with open(os.path.join(d, 'permissions.p7s'), 'rb') as f:
        content = signed_content(f.read())
    with open(os.path.join(d, 'permissions.xml'), 'rb') as f:
        assert content == f.read()
    grant = ET.fromstring(content).find('permissions/grant')
    ordered = [(child.tag, [int(i.text) for i in child.iterfind('domains/id')],
                child.find('publish') is not None, child.find('subscribe') is not None)
               for child in grant if child.tag.endswith('_rule')]
    assert ordered[:3] == [('allow_rule', [10], False, True), ('deny_rule', [10], True, False),
                           ('allow_rule', [10], True, False)]
    assert ordered[-1] == ('allow_rule', [10, 20, 30], True, True)
    denied = [t.text for t in grant.find('deny_rule').iter('topic')]
    assert denied == ['rt/plc/reference', 'rt/robot/reference', 'rt/attack/truth']
    assert grant.findtext('default') == 'DENY'
    assert grant.findtext('subject_name') == 'CN=/cell/robot'


def test_governance_refuses_strangers_and_encrypts(keystore):
    out, _ = keystore
    root = ET.parse(os.path.join(out, 'enclaves', 'governance.xml')).getroot()
    rule = root.find('domain_access_rules/domain_rule')
    assert [int(e.text) for e in rule.iterfind('domains/id')] == [10, 20, 30]
    assert rule.findtext('allow_unauthenticated_participants') == 'false'
    assert rule.findtext('enable_join_access_control') == 'true'
    assert rule.findtext('topic_access_rules/topic_rule/data_protection_kind') == 'ENCRYPT'


def test_rerun_keeps_authority_and_keys_and_force_replaces_them(tmp_path, policy):
    out = str(tmp_path / 'ks')
    first = build_keystore(policy, out)
    again = build_keystore(policy, out)
    assert again['authority']['sha256'] == first['authority']['sha256']
    assert not again['authority']['created'] and all(e['reused_key'] for e in again['enclaves'])
    fresh = build_keystore(policy, out, force=True)
    assert fresh['authority']['sha256'] != first['authority']['sha256']
    assert not any(e['reused_key'] for e in fresh['enclaves'])


def test_enclaves_removed_from_the_policy_are_deleted(tmp_path, policy):
    out = str(tmp_path / 'ks')
    build_keystore(policy, out)
    smaller = policy.__class__(policy.common_name, policy.forged_common_name,
                               policy.validity_days, policy.domains, policy.infrastructure,
                               tuple(e for e in policy.enclaves if e.path != '/cell/plc'))
    manifest = build_keystore(smaller, out)
    assert manifest['removed'] == ['/cell/plc']
    assert not os.path.exists(enclave_dir(out, '/cell/plc'))


def test_checker_reports_missing_files_and_expired_certificates(tmp_path, policy):
    out = str(tmp_path / 'ks')
    build_keystore(policy, out)
    later = datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(days=4000)
    assert any('certificate valid now' in t for t in
               failures(check_enclave(out, '/cell/plc', 10, now=later)))
    os.remove(os.path.join(enclave_dir(out, '/cell/plc'), 'key.pem'))
    bad = failures(check_enclave(out, '/cell/plc', 10))
    assert len(bad) == 1 and 'key.pem' in bad[0]


def test_checker_rejects_an_identity_from_another_authority(tmp_path, policy):
    out = str(tmp_path / 'ks')
    build_keystore(policy, out)
    forged = enclave_dir(os.path.join(out, 'loot'), '/attacker/forged')
    target = enclave_dir(out, '/cell/plc')
    for name in ('cert.pem', 'key.pem'):
        shutil.copy(os.path.join(forged, name), os.path.join(target, name))
    bad = failures(check_enclave(out, '/cell/plc', 10))
    assert any('CN=/cell/plc' in t for t in bad)
    assert any('signed by the identity CA' in t for t in bad)


@pytest.mark.skipif(not OPENSSL, reason='openssl not installed')
def test_checker_detects_tampered_permissions(tmp_path, policy):
    out = str(tmp_path / 'ks')
    build_keystore(policy, out)
    path = os.path.join(enclave_dir(out, '/cell/plc'), 'permissions.p7s')
    with open(path, 'rb') as f:
        data = f.read()
    with open(path, 'wb') as f:
        f.write(data.replace(b'rt/plc/reference', b'rt/rob/reference', 1))
    bad = failures(check_enclave(out, '/cell/plc', 10))
    assert any('permissions.p7s signed' in t for t in bad)


def test_command_line_tools(tmp_path, capsys):
    out = str(tmp_path / 'ks')
    assert make_keystore.main(['--out', out]) == 0
    assert 'keystore written' in capsys.readouterr().out
    env = {'ROS_SECURITY_KEYSTORE': out, 'ROS_SECURITY_ENCLAVE_OVERRIDE': '/cell/robot',
           'ROS_DOMAIN_ID': '20', 'ROS_SECURITY_ENABLE': 'true',
           'ROS_SECURITY_STRATEGY': 'Enforce'}
    assert check_keystore.main([], env) == 0
    assert 'PASS /cell/robot' in capsys.readouterr().out
    env['ROS_SECURITY_ENCLAVE_OVERRIDE'] = '/monitor/ids'
    env['ROS_DOMAIN_ID'] = '10'
    assert check_keystore.main([], env) == 1
    assert check_keystore.main(['--all', '--domain', '10'], env) == 1
    assert check_keystore.main(['--keystore', str(tmp_path / 'none')], {}) == 1