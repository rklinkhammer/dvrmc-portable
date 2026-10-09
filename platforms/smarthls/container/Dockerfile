FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
# Vendor Ubuntu 24.04 prerequisite list; libgcc1 is replaced by libgcc-s1.
COPY packages.txt /tmp/microchip-packages.txt
RUN dpkg --add-architecture i386 && apt-get update && \
    xargs -a /tmp/microchip-packages.txt apt-get install -y --no-install-recommends && \
    apt-get install -y --no-install-recommends build-essential cmake python3 \
      ca-certificates file git procps tcsh unzip xauth xvfb x11-utils libxt6 \
      libxmu6 libxpm4 libncurses6 libtinfo6 && \
    rm -rf /var/lib/apt/lists/*
# Synopsys licensing uses /usr/tmp for daemon lock files.
RUN mkdir -p /usr/tmp && chmod 1777 /usr/tmp
ENV PATH="/opt/microchip/SmartHLS/SmartHLS/bin:/opt/microchip/Libero_SoC/Designer/bin:/opt/microchip/Libero_SoC/QuestaSim_Pro/bin:${PATH}"
ENV QT_X11_NO_MITSHM=1
WORKDIR /workspace
CMD ["bash"]
