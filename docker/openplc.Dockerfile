FROM debian:bookworm-slim

ARG OPENPLC_REF=v3

RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential autoconf automake bison flex \
        python3 python3-pip libssl-dev sqlite3 libsqlite3-dev \
        pkg-config ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${OPENPLC_REF} \
        https://github.com/thiagoralves/OpenPLC_v3 /opt/openplc \
    && cd /opt/openplc \
    && echo win32 > /dev/null \
    && ./install.sh linux

COPY plc/ /opt/openplc/webserver/st_files/cell/
COPY docker/openplc-entrypoint.sh /openplc-entrypoint.sh
RUN chmod +x /openplc-entrypoint.sh

EXPOSE 502 8080
WORKDIR /opt/openplc
ENTRYPOINT ["/openplc-entrypoint.sh"]