from dataclasses import dataclass
import email
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ids_security.keystore import (common_name, enclave_dir, load_cert, load_key, same_key,
                                   signed_by, TIME_FORMAT, utc_now, validity)

REQUIRED = ('identity_ca.cert.pem', 'cert.pem', 'key.pem', 'permissions_ca.cert.pem',
            'governance.p7s', 'permissions.p7s')


@dataclass(frozen=True)
class Finding:
    ok: bool
    text: str


def signed_content(data):
    message = email.message_from_bytes(data)
    if not message.is_multipart():
        raise ValueError('not an S/MIME multipart/signed document')
    body = message.get_payload(0).get_payload(decode=True)
    return body.replace(b'\r\n', b'\n')


def verify_smime(path, ca_path, openssl='openssl'):
    if shutil.which(openssl) is None:
        return None
    result = subprocess.run([openssl, 'smime', '-verify', '-purpose', 'any', '-in', path,
                             '-CAfile', ca_path, '-out', os.devnull],
                            capture_output=True, text=True)
    return result.returncode == 0


def _domain_ids(element, path):
    return {int(e.text) for e in element.iterfind(path)}


def _parse_time(text):
    import datetime
    return datetime.datetime.strptime(text.strip(), TIME_FORMAT)


def find_enclaves(keystore):
    base = os.path.join(keystore, 'enclaves')
    found = []
    for d, _, files in os.walk(base):
        if 'key.pem' in files:
            found.append('/' + os.path.relpath(d, base).replace(os.sep, '/'))
    return sorted(found)


def check_enclave(keystore, enclave, domain_id=None, now=None, openssl='openssl'):
    now = (now or utc_now()).replace(tzinfo=None)
    directory = enclave_dir(keystore, enclave)
    out = []

    def add(ok, text):
        out.append(Finding(bool(ok), text))

    missing = [f for f in REQUIRED if not os.path.isfile(os.path.join(directory, f))]
    add(not missing, f'files present in {directory}' if not missing
        else f'missing in {directory}: {", ".join(missing)}')
    if missing:
        return out
    path = lambda name: os.path.join(directory, name)  # noqa: E731
    cert, key = load_cert(path('cert.pem')), load_key(path('key.pem'))
    identity_ca = load_cert(path('identity_ca.cert.pem'))
    add(cert.subject == common_name(enclave), f'certificate subject is CN={enclave}')
    add(signed_by(cert, identity_ca), 'certificate signed by the identity CA in the enclave')
    start, end = validity(cert)
    add(start <= now <= end, f'certificate valid now ({start:%Y-%m-%d} to {end:%Y-%m-%d})')
    add(same_key(cert, key), 'private key matches the certificate')
    for name in ('permissions.p7s', 'governance.p7s'):
        verified = verify_smime(path(name), path('permissions_ca.cert.pem'), openssl)
        if verified is None:
            add(True, f'{name}: signature not checked (openssl not found)')
        else:
            add(verified, f'{name} signed by the permissions CA')
    try:
        permissions = ET.fromstring(signed_content(open(path('permissions.p7s'), 'rb').read()))
        governance = ET.fromstring(signed_content(open(path('governance.p7s'), 'rb').read()))
    except (ValueError, ET.ParseError, IndexError) as e:
        add(False, f'cannot read the signed XML: {e}')
        return out
    grants = permissions.findall('permissions/grant')
    subjects = [g.findtext('subject_name') for g in grants]
    add(f'CN={enclave}' in subjects, f'permissions grant for CN={enclave}')
    grant = next((g for g in grants if g.findtext('subject_name') == f'CN={enclave}'), None)
    if grant is not None:
        before = _parse_time(grant.findtext('validity/not_before'))
        after = _parse_time(grant.findtext('validity/not_after'))
        add(before <= now <= after, 'permissions valid now')
        add(grant.findtext('default') == 'DENY', 'permissions default is DENY')
        granted = _domain_ids(grant, './/domains/id')
        governed = _domain_ids(governance, 'domain_access_rules/domain_rule/domains/id')
        if domain_id is not None:
            add(domain_id in granted, f'permissions cover domain {domain_id} '
                f'(granted {sorted(granted)})')
            add(domain_id in governed, f'governance covers domain {domain_id} '
                f'(governed {sorted(governed)})')
    rule = governance.find('domain_access_rules/domain_rule')
    add(rule is not None and rule.findtext('allow_unauthenticated_participants') == 'false',
        'governance refuses unauthenticated participants')
    return out