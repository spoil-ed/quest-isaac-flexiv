FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ubuntu

RUN if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then \
        sed -i \
          -e "s|http://archive.ubuntu.com/ubuntu|${APT_MIRROR}|g" \
          -e "s|http://security.ubuntu.com/ubuntu|${APT_MIRROR}|g" \
          -e 's/^Components:.*/Components: main restricted universe multiverse/' \
          /etc/apt/sources.list.d/ubuntu.sources; \
    fi \
    && if [ -f /etc/apt/sources.list ]; then \
        sed -i \
          -e "s|http://archive.ubuntu.com/ubuntu|${APT_MIRROR}|g" \
          -e "s|http://security.ubuntu.com/ubuntu|${APT_MIRROR}|g" \
          -e 's/ noble main$/ noble main restricted universe multiverse/' \
          /etc/apt/sources.list; \
    fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        dbus-x11 \
        fonts-dejavu-core \
        freeglut3-dev \
        fuse3 \
        gocryptfs \
        iproute2 \
        iputils-ping \
        libasound2t64 \
        libatomic1 \
        libcap2-bin \
        libdrm2 \
        libegl1 \
        libgbm1 \
        libgl1 \
        libglu1-mesa \
        libgomp1 \
        libgtk-3-0t64 \
        libnss3 \
        libpulse0 \
        libqt5xml5 \
        libx11-xcb1 \
        libxcb-icccm4 \
        libxcb-image0 \
        libxcb-keysyms1 \
        libxcb-render-util0 \
        libxcb-shape0 \
        libxcb-xinerama0 \
        libxcb-xinput0 \
        libxcb-xkb1 \
        libxcomposite1 \
        libxdamage1 \
        libxkbcommon-x11-0 \
        libxrandr2 \
        libxtst6 \
        net-tools \
        openbox \
        procps \
        x11-utils \
        x11vnc \
        xauth \
        xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid 1000 flexiv \
    && useradd --uid 1000 --gid 1000 --create-home --shell /bin/bash flexiv

COPY entrypoint.sh /usr/local/bin/flexiv-studio-entrypoint
COPY healthcheck.sh /usr/local/bin/flexiv-studio-healthcheck
RUN chmod 0755 /usr/local/bin/flexiv-studio-entrypoint /usr/local/bin/flexiv-studio-healthcheck \
    && setcap cap_sys_nice=ep /sbin/capsh

ENV DISPLAY=:1 \
    HOME=/tmp/flexiv-home \
    NVIDIA_DRIVER_CAPABILITIES=all \
    NVIDIA_VISIBLE_DEVICES=all \
    QTWEBENGINE_DISABLE_SANDBOX=1 \
    QT_X11_NO_MITSHM=1

WORKDIR /opt/flexiv-studio
EXPOSE 5900

HEALTHCHECK --interval=10s --timeout=5s --start-period=45s --retries=6 \
    CMD ["flexiv-studio-healthcheck"]

ENTRYPOINT ["flexiv-studio-entrypoint"]
