from dataclasses import dataclass
import datetime
import hashlib
import os
import shutil
import xml.etree.ElementTree as ET

from cryptography import x509
from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.serialization import pkcs7
from cryptography.x509.oid import NameOID
import yaml

from ids_security.policy import dds_topic, rules

GOVERNANCE_XSD = 'http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_governance.xsd'
PERMISSIONS_XSD = 'http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_permissions.xsd'
XSI = 'http://www.w3.org/2001/XMLSchema-instance'
ENCLAVE_FILES = ('identity_ca.cert.pem', 'permissions_ca.cert.pem', 'governance.p7s', 'cert.pem',
                 'key.pem', 'permissions.xml', 'permissions.p7s')
CA_NAMES = ('ca', 'identity_ca', 'permissions_ca')
TIME_FORMAT = '%Y-%m-%dT%H:%M:%S'


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).replace(microsecond=0)


def common_name(cn):
    return x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, cn)])


def new_key():
    return ec.generate_private_key(ec.SECP256R1())


def build_certificate(subject, public_key, issuer_name, issuer_key, ca, days, now):
    builder = (x509.CertificateBuilder()
               .subject_name(subject)
               .issuer_name(issuer_name)
               .public_key(public_key)
               .serial_number(x509.random_serial_number())
               .not_valid_before((now - datetime.timedelta(days=1)).replace(tzinfo=None))
               .not_valid_after((now + datetime.timedelta(days=days)).replace(tzinfo=None))
               .add_extension(x509.BasicConstraints(ca=ca, path_length=1 if ca else None),
                              critical=ca))
    return builder.sign(issuer_key, hashes.SHA256())


def cert_pem(cert):
    return cert.public_bytes(serialization.Encoding.PEM)


def key_pem(key):
    return key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                             serialization.NoEncryption())


def load_cert(path):
    with open(path, 'rb') as f:
        return x509.load_pem_x509_certificate(f.read())


def load_key(path):
    with open(path, 'rb') as f:
        return serialization.load_pem_private_key(f.read(), None)


def fingerprint(cert):
    return hashlib.sha256(cert.public_bytes(serialization.Encoding.DER)).hexdigest()


def validity(cert):
    if hasattr(cert, 'not_valid_before_utc'):
        return (cert.not_valid_before_utc.replace(tzinfo=None),
                cert.not_valid_after_utc.replace(tzinfo=None))
    return cert.not_valid_before, cert.not_valid_after


def signed_by(cert, issuer_cert):
    try:
        issuer_cert.public_key().verify(cert.signature, cert.tbs_certificate_bytes,
                                        ec.ECDSA(cert.signature_hash_algorithm))
    except (InvalidSignature, TypeError, ValueError):
        return False
    return cert.issuer == issuer_cert.subject


def same_key(cert, key):
    return cert.public_key().public_numbers() == key.public_key().public_numbers()


def sign_smime(cert, key, data):
    options = [pkcs7.PKCS7Options.Text, pkcs7.PKCS7Options.DetachedSignature]
    return (pkcs7.PKCS7SignatureBuilder().set_data(data)
            .add_signer(cert, key, hashes.SHA256())
            .sign(serialization.Encoding.SMIME, options))


@dataclass
class Authority:
    cert: object
    key: object

    @property
    def name(self):
        return self.cert.subject

    @classmethod
    def create(cls, cn, days, now):
        key = new_key()
        return cls(build_certificate(common_name(cn), key.public_key(), common_name(cn), key,
                                     True, days, now), key)

    @classmethod
    def load_or_create(cls, cert_path, key_path, cn, days, now):
        if os.path.isfile(cert_path) and os.path.isfile(key_path):
            authority = cls(load_cert(cert_path), load_key(key_path))
            if same_key(authority.cert, authority.key) and authority.name == common_name(cn):
                return authority, False
        return cls.create(cn, days, now), True

    def issue(self, cn, days, now):
        key = new_key()
        return build_certificate(common_name(cn), key.public_key(), self.name, self.key, False,
                                 days, now), key

    def sign(self, data):
        return sign_smime(self.cert, self.key, data)


def _xml(root):
    ET.indent(root, space='  ')
    return ('<?xml version="1.0" encoding="UTF-8"?>\n'
            + ET.tostring(root, encoding='unicode') + '\n').encode()


def _dds_root(schema):
    ET.register_namespace('xsi', XSI)
    return ET.Element('dds', {f'{{{XSI}}}noNamespaceSchemaLocation': schema})


def _domains(parent, ids):
    domains = ET.SubElement(parent, 'domains')
    for domain_id in sorted(set(ids)):
        ET.SubElement(domains, 'id').text = str(domain_id)


def governance_xml(domain_ids):
    root = _dds_root(GOVERNANCE_XSD)
    rule = ET.SubElement(ET.SubElement(root, 'domain_access_rules'), 'domain_rule')
    _domains(rule, domain_ids)
    for tag, text in (('allow_unauthenticated_participants', 'false'),
                      ('enable_join_access_control', 'true'),
                      ('discovery_protection_kind', 'ENCRYPT'),
                      ('liveliness_protection_kind', 'ENCRYPT'),
                      ('rtps_protection_kind', 'SIGN')):
        ET.SubElement(rule, tag).text = text
    topic = ET.SubElement(ET.SubElement(rule, 'topic_access_rules'), 'topic_rule')
    for tag, text in (('topic_expression', '*'),
                      ('enable_discovery_protection', 'true'),
                      ('enable_liveliness_protection', 'true'),
                      ('enable_read_access_control', 'true'),
                      ('enable_write_access_control', 'true'),
                      ('metadata_protection_kind', 'ENCRYPT'),
                      ('data_protection_kind', 'ENCRYPT')):
        ET.SubElement(topic, tag).text = text
    return _xml(root)


def _criteria(rule, action, patterns):
    if not patterns:
        return
    topics = ET.SubElement(ET.SubElement(rule, action), 'topics')
    for p in patterns:
        ET.SubElement(topics, 'topic').text = dds_topic(p)


def _rule(grant_element, rule):
    element = ET.SubElement(grant_element, rule.kind)
    _domains(element, rule.domains)
    _criteria(element, 'publish', rule.publish)
    _criteria(element, 'subscribe', rule.subscribe)


def permissions_xml(policy, enclave, not_before, not_after):
    root = _dds_root(PERMISSIONS_XSD)
    grant = ET.SubElement(ET.SubElement(root, 'permissions'), 'grant', {'name': enclave.path})
    ET.SubElement(grant, 'subject_name').text = f'CN={enclave.path}'
    window = ET.SubElement(grant, 'validity')
    ET.SubElement(window, 'not_before').text = not_before.strftime(TIME_FORMAT)
    ET.SubElement(window, 'not_after').text = not_after.strftime(TIME_FORMAT)
    for rule in rules(policy, enclave):
        _rule(grant, rule)
    ET.SubElement(grant, 'default').text = 'DENY'
    return _xml(root)


def _write(path, data, private=False):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(data)
    try:
        os.chmod(path, 0o600 if private else 0o644)
    except OSError:
        pass


def _save_authority(authority, directory, names):
    for name in names:
        _write(os.path.join(directory, 'public', f'{name}.cert.pem'), cert_pem(authority.cert))
        _write(os.path.join(directory, 'private', f'{name}.key.pem'), key_pem(authority.key),
               private=True)


def enclave_dir(root, enclave_path):
    return os.path.join(root, 'enclaves', *enclave_path.lstrip('/').split('/'))


def _reuse_identity(directory, enclave_path, authority, now):
    cert_path, key_path = (os.path.join(directory, n) for n in ('cert.pem', 'key.pem'))
    if not (os.path.isfile(cert_path) and os.path.isfile(key_path)):
        return None
    try:
        cert, key = load_cert(cert_path), load_key(key_path)
    except (ValueError, TypeError):
        return None
    start, end = validity(cert)
    if (signed_by(cert, authority.cert) and same_key(cert, key)
            and cert.subject == common_name(enclave_path)
            and start <= now.replace(tzinfo=None) <= end):
        return cert, key
    return None


def _write_enclave(policy, enclave, root, authority, governance_p7s, now):
    directory = enclave_dir(root, enclave.path)
    identity = _reuse_identity(directory, enclave.path, authority, now)
    if os.path.isdir(directory):
        for name in os.listdir(directory):
            path = os.path.join(directory, name)
            if os.path.isfile(path) and name not in ('cert.pem', 'key.pem'):
                os.remove(path)
    cert, key = identity or authority.issue(enclave.path, policy.validity_days, now)
    _write(os.path.join(directory, 'cert.pem'), cert_pem(cert))
    _write(os.path.join(directory, 'key.pem'), key_pem(key), private=True)
    _write(os.path.join(directory, 'identity_ca.cert.pem'), cert_pem(authority.cert))
    _write(os.path.join(directory, 'permissions_ca.cert.pem'), cert_pem(authority.cert))
    _write(os.path.join(directory, 'governance.p7s'), governance_p7s)
    xml = permissions_xml(policy, enclave, *validity(cert))
    _write(os.path.join(directory, 'permissions.xml'), xml)
    _write(os.path.join(directory, 'permissions.p7s'), authority.sign(xml))
    return {'path': enclave.path, 'store': enclave.store, 'ca': enclave.ca,
            'domains': sorted(g.domain_id for g in enclave.grants),
            'cert_sha256': fingerprint(cert), 'reused_key': identity is not None}


def _remove_stale(root, keep):
    base = os.path.join(root, 'enclaves')
    removed = []
    if not os.path.isdir(base):
        return removed
    for d, _, files in os.walk(base, topdown=False):
        if 'key.pem' in files:
            path = '/' + os.path.relpath(d, base).replace(os.sep, '/')
            if path not in keep:
                shutil.rmtree(d)
                removed.append(path)
    return removed


def build_keystore(policy, out_dir, now=None, force=False):
    now = now or utc_now()
    if force and os.path.isdir(out_dir):
        for name in os.listdir(out_dir):
            path = os.path.join(out_dir, name)
            shutil.rmtree(path) if os.path.isdir(path) else os.remove(path)
    loot = os.path.join(out_dir, 'loot')
    authority, created = Authority.load_or_create(
        os.path.join(out_dir, 'public', 'ca.cert.pem'),
        os.path.join(out_dir, 'private', 'ca.key.pem'),
        policy.common_name, policy.validity_days, now)
    _save_authority(authority, out_dir, CA_NAMES)
    forged, forged_created = Authority.load_or_create(
        os.path.join(loot, 'forged_ca', 'public', 'ca.cert.pem'),
        os.path.join(loot, 'forged_ca', 'private', 'ca.key.pem'),
        policy.forged_common_name, policy.validity_days, now)
    _save_authority(forged, os.path.join(loot, 'forged_ca'), ('ca',))
    governance = governance_xml(policy.domain_ids)
    _write(os.path.join(out_dir, 'enclaves', 'governance.xml'), governance)
    signed = {'cell': authority.sign(governance), 'forged': forged.sign(governance)}
    _write(os.path.join(out_dir, 'enclaves', 'governance.p7s'), signed['cell'])
    authorities = {'cell': authority, 'forged': forged}
    roots = {'keystore': out_dir, 'loot': loot}
    enclaves = [_write_enclave(policy, e, roots[e.store], authorities[e.ca], signed[e.ca], now)
                for e in policy.enclaves]
    removed = []
    for store, root in roots.items():
        removed += _remove_stale(root, {e.path for e in policy.in_store(store)})
    manifest = {
        'created': now.strftime(TIME_FORMAT) + 'Z',
        'authority': {'common_name': policy.common_name, 'sha256': fingerprint(authority.cert),
                      'created': created},
        'forged_authority': {'common_name': policy.forged_common_name,
                             'sha256': fingerprint(forged.cert), 'created': forged_created},
        'domains': dict(policy.domains),
        'enclaves': enclaves,
        'removed': removed,
    }
    _write(os.path.join(out_dir, 'manifest.yaml'),
           yaml.safe_dump(manifest, sort_keys=False).encode())
    return manifest