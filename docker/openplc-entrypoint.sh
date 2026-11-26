#!/bin/bash
set -e
cd /opt/openplc/webserver
if [ -f st_files/cell/cell.st ]; then
  cp st_files/cell/cell.st st_files/cell.st
  ./scripts/compile_program.sh cell.st || echo "compile pending: load cell.st from the web UI"
fi
exec python3 webserver.py