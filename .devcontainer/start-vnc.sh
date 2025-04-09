#!/bin/bash

# Configurações de ambiente
export USER=${USER:-ubuntu}
export HOME=/home/$USER
export DISPLAY=:1

# Mata instâncias antigas
vncserver -kill $DISPLAY &> /dev/null || true

# Limpa locks e arquivos antigos
rm -rf /tmp/.X1-lock /tmp/.X11-unix/X1 $HOME/.vnc/*

# Cria um xstartup básico se não existir
if [ ! -f "$HOME/.vnc/xstartup" ]; then
    mkdir -p "$HOME/.vnc"
    cat <<EOF > "$HOME/.vnc/xstartup"
#!/bin/sh
exec startxfce4
EOF
    chmod +x "$HOME/.vnc/xstartup"
fi

# Inicia o servidor VNC com TurboVNC sem senha
vncserver $DISPLAY -geometry 1920x1200 -depth 24 -securitytypes None

# Aguarda o ambiente gráfico inicializar
sleep 3

# Ajusta resolução automaticamente
RES_WIDTH=$(xdpyinfo | awk '/dimensions:/ {print $2}' | cut -d 'x' -f 1)
RES_HEIGHT=$(xdpyinfo | awk '/dimensions:/ {print $2}' | cut -d 'x' -f 2)

# Define resolução mínima aceitável
if [ -z "$RES_WIDTH" ] || [ "$RES_WIDTH" -lt 800 ]; then
  RES_WIDTH=1280
fi
if [ -z "$RES_HEIGHT" ] || [ "$RES_HEIGHT" -lt 600 ]; then
  RES_HEIGHT=800
fi

RESOLUTION="${RES_WIDTH}x${RES_HEIGHT}"
xrandr -s $RESOLUTION || echo "[Aviso] xrandr não conseguiu aplicar a resolução $RESOLUTION"

# Garante que a porta 6080 esteja livre
sudo fuser -k 6080/tcp || true

# Inicia o noVNC na porta 6080 apontando para o VNC (5901)
websockify --web=/usr/share/novnc --wrap-mode=ignore 6080 localhost:5901 &

# Aguarda e inicia os apps (VSCode, terminais, etc)
sleep 5
bash "$HOME/start-apps.sh"

# Mantém container rodando
tail -f /dev/null
