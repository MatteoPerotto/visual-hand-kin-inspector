FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04
LABEL org.opencontainers.image.title="Ubuntu 20.04"
LABEL org.opencontainers.image.authors="Nicola A. Piga <nicola.piga@iit.it>"

# Use /bin/bash instead of /bin/sh
SHELL ["/bin/bash", "-c"]

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Set the locale
RUN apt update && \
    apt install -y -qq locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
RUN apt update && \
    apt install --no-install-recommends -y -qq apt-utils build-essential ca-certificates cmake cmake-curses-gui curl emacs-nox git glmark2 gnupg2 htop iputils-ping jq lsb-release mesa-utils nano psmisc python3-virtualenv sudo unzip vim wget zip && \
    rm -rf /var/lib/apt/lists/*

# Install additional dependencies
RUN apt update && \
    apt install --no-install-recommends -y -qq libassimp-dev libglfw3-dev libglew-dev libglm-dev libeigen3-dev libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

# Install GitHub cli
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
    apt update && \
    apt install --no-install-recommends -y -qq gh && \
    rm -rf /var/lib/apt/lists/*

# Create user with passwordless sudo
RUN useradd -l -G sudo -md /home/user -s /bin/bash -p user user && \
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Switch to user
USER user

# Configure GitHub
git config --global user.name "User"
git config --gloval user.email "user@mail.com"

# Build robotology-superbuild
RUN git clone https://github.com/robotology/robotology-superbuild && \
    cd robotology-superbuild && mkdir build && cd build && \
    cmake -DROBOTOLOGY_USES_GAZEBO=OFF -DROBOTOLOGY_ENABLE_DYNAMICS=ON . && \
    make

# Build superimpose-mesh-lib
RUN /home/user/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh && \
    git clone https://github.com/robotology/superimpose-mesh-lib && \
    cd superimpose-mesh-lib && mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/home/user/robotology-superbuild/build/install .. && \
    make install

# Configure emacs
RUN echo "(setq-default indent-tabs-mode nil)" >> /home/user/.emacs.el && \
    echo "(setq-default tab-width 4)" >> /home/user/.emacs.el && \
    echo "(setq make-backup-files nil)" >> /home/user/.emacs.el && \
    echo "(setq auto-save-default nil)" >> /home/user/.emacs.el && \
    echo "(setq c-default-style \"linux\"" >> /home/user/.emacs.el && \
    echo "      c-basic-offset 4)" >> /home/user/.emacs.el && \
    echo "(global-subword-mode 1)" >> /home/user/.emacs.el && \
    echo "(add-hook 'before-save-hook 'delete-trailing-whitespace)" >> /home/user/.emacs.el && \
    echo "(custom-set-variables '(custom-enabled-themes '(tango-dark)))" >> /home/user/.emacs.el && \
    echo "(custom-set-faces)" >> /home/user/.emacs.elx

# Configure robotology-superbuild
RUN echo "source /home/user/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> /home/user/.bashrc

# Launch bash from /home/user
WORKDIR /home/user
CMD ["bash"]
