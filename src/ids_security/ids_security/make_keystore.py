import argparse
import sys

from ids_security.keystore import build_keystore
from ids_security.policy import load_policy


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Create or update the SROS2 keystore of the cell from the policy file.')
    parser.add_argument('--out', default='/keystore', help='keystore folder to write')
    parser.add_argument('--policy', default=None, help='policy file (default: installed)')
    parser.add_argument('--force', action='store_true',
                        help='delete the keystore first and create new authorities')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    manifest = build_keystore(load_policy(args.policy), args.out, force=args.force)
    authority = manifest['authority']
    print(f"authority {authority['common_name']} "
          f"({'new' if authority['created'] else 'kept'}), sha256 {authority['sha256'][:16]}")
    for e in manifest['enclaves']:
        print(f"  {e['path']:<20} {e['store']:<9} ca {e['ca']:<7} domains {e['domains']} "
              f"key {'kept' if e['reused_key'] else 'new'}")
    for path in manifest['removed']:
        print(f'  removed stale enclave {path}')
    print(f'keystore written to {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())