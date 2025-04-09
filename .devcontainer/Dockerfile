FROM osrf/ros:noetic-desktop-full

# Argumentos para criar usuário não-root
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Cria usuário e adiciona permissões sudo
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Alterna para o novo usuário
USER $USERNAME

# Adiciona usuário ao grupo video para acesso a webcam
RUN sudo usermod --append --groups video $USERNAME

# Atualiza pacotes e instala dependências ROS
RUN sudo apt update && sudo apt upgrade -y && \
    sudo apt install -y git nano python-is-python3 \
    ros-noetic-ros-tutorials \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    liburdfdom-tools \
    ros-noetic-effort-controllers \
    ros-noetic-ros-control ros-noetic-ros-controllers \
    python3-catkin-tools \
    ros-noetic-moveit

# Atualiza rosdep
RUN sudo rosdep update

# Retorna para root para instalar ambiente gráfico
USER root

# Instala XFCE minimalista + ferramentas gráficas necessárias
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
    xfce4 xfce4-terminal \
    novnc websockify xterm x11vnc wget curl net-tools \
    python3-pip libgl1-mesa-dri libgl1-mesa-glx dbus-x11 mesa-utils \
    x11-utils x11-xserver-utils wmctrl xdotool \
    && apt-get clean

# Keyring para o vscode
RUN apt-get update && apt-get install -y \
    gnome-keyring \
    libsecret-1-0 \
    libsecret-1-dev


## Instala TurboVNC
RUN wget https://github.com/TurboVNC/turbovnc/releases/download/3.1.1/turbovnc_3.1.1_amd64.deb && \
    dpkg -i turbovnc_3.1.1_amd64.deb && \
    rm turbovnc_3.1.1_amd64.deb

# Instala VSCode
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg && \
    install -o root -g root -m 644 microsoft.gpg /usr/share/keyrings/ && \
    sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] \
    https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list' && \
    apt update && apt install -y code && \
    rm microsoft.gpg

# Cria diretório .vnc e copia script de inicialização
RUN mkdir -p /home/${USERNAME}/.vnc && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}


# Copia o script para o diretório do usuário
COPY start-vnc.sh /home/${USERNAME}/start-vnc.sh
RUN chown ${USERNAME}:${USERNAME} /home/${USERNAME}/start-vnc.sh && \
    chmod +x /home/${USERNAME}/start-vnc.sh

# Copia o script start-apps para o diretório do usuário
COPY start-apps.sh /home/${USERNAME}/start-apps.sh
RUN chown ${USERNAME}:${USERNAME} /home/${USERNAME}/start-apps.sh && \
    chmod +x /home/${USERNAME}/start-apps.sh


# Exporta portas
EXPOSE 5901 6080

# Adiciona ROS setup e init.bash ao bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /projeto1_PosDoc/init.bash" >> /home/$USERNAME/.bashrc && \
    chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

# Alterna para usuário não-root
USER $USERNAME

# Comando default
CMD ["/usr/local/bin/start-vnc.sh"]
