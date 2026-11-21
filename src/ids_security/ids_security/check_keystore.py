import argparse
import os
import sys

from ids_security.checks import check_enclave, find_enclaves


def parse_args(argv=None, env=os.environ):
    parser = argparse.ArgumentParser(
        description='Check the SROS2 files this container will use (default: from the '
                    'ROS_SECURITY_* environment).')
    parser.add_argument('--keystore', default=env.get('ROS_SECURITY_KEYSTORE', '/keystore'))
    parser.add_argument('--enclave', default=env.get('ROS_SECURITY_ENCLAVE_OVERRIDE'))
    parser.add_argument('--all', action='store_true', help='check every enclave found')
    parser.add_argument('--domain', type=int,
                        default=int(env['ROS_DOMAIN_ID']) if env.get('ROS_DOMAIN_ID') else None)
    return parser.parse_args(argv)


def main(argv=None, env=os.environ):
    args = parse_args(argv, env)
    enclaves = find_enclaves(args.keystore) if args.all or not args.enclave else [args.enclave]
    if not enclaves:
        print(f'no enclaves under {args.keystore}/enclaves')
        return 1
    failed = 0
    print(f"security {'enabled' if env.get('ROS_SECURITY_ENABLE') == 'true' else 'disabled'}, "
          f"strategy {env.get('ROS_SECURITY_STRATEGY', '-')}, keystore {args.keystore}")
    for enclave in enclaves:
        findings = check_enclave(args.keystore, enclave, args.domain)
        bad = [f for f in findings if not f.ok]
        failed += bool(bad)
        print(f"{'PASS' if not bad else 'FAIL'} {enclave}")
        for f in findings:
            print(f"  {'ok  ' if f.ok else 'FAIL'} {f.text}")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())