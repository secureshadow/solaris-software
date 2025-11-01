#!/usr/bin/env bash
set -euo pipefail

echo "🚀 Iniciando migración de Forgejo → GitHub (sin push automático)..."
echo

# --- CONFIGURACIÓN INICIAL ---
MAIN_REMOTE="git@github.com:Software-Solaris/solaris-software.git"
SPP_PORTS_REMOTE="git@github.com:Software-Solaris/spp-ports.git"
SPP_REMOTE="git@github.com:Software-Solaris/solaris-packet-protocol.git"
ESP_IDF_REMOTE="https://github.com/espressif/esp-idf.git"

# --- 1️⃣ ACTUALIZAR .gitmodules ---
echo "🔧 Reescribiendo archivo .gitmodules con las nuevas URLs..."
cat > .gitmodules <<EOF
[submodule "solaris-v1/external/spp"]
    path = solaris-v1/external/spp
    url = ${SPP_REMOTE}
[submodule "solaris-v1/external/spp-ports"]
    path = solaris-v1/external/spp-ports
    url = ${SPP_PORTS_REMOTE}
[submodule "solaris-v1/external/esp-idf"]
    path = solaris-v1/external/esp-idf
    url = ${ESP_IDF_REMOTE}
EOF

echo "✅ .gitmodules actualizado."
echo
echo "📄 Nuevo contenido de .gitmodules:"
cat .gitmodules
echo

# --- 2️⃣ SINCRONIZAR SUBMÓDULOS ---
echo "🔄 Sincronizando submódulos..."
git submodule sync --recursive
echo "✅ Submódulos sincronizados."
echo

# --- 3️⃣ CONFIGURAR REMOTOS ---
echo "➡️ Configurando remote del repositorio principal..."
git remote set-url origin "$MAIN_REMOTE"
git remote -v
echo "✅ Repositorio principal configurado."
echo

echo "➡️ Configurando submódulo SPP..."
cd solaris-v1/external/spp
git remote set-url origin "$SPP_REMOTE"
git remote -v
cd - > /dev/null
echo "✅ Submódulo SPP configurado."
echo

echo "➡️ Configurando submódulo SPP-PORTS..."
cd solaris-v1/external/spp-ports
git remote set-url origin "$SPP_PORTS_REMOTE"
git remote -v
cd - > /dev/null
echo "✅ Submódulo SPP-PORTS configurado."
echo

# --- 4️⃣ ACTUALIZAR CONTENIDO DE SUBMÓDULOS ---
echo "📦 Actualizando submódulos con el nuevo origen..."
git submodule update --init --recursive
echo "✅ Submódulos actualizados."
echo

# --- 5️⃣ PROBAR CONEXIÓN SSH A GITHUB ---
echo "🔐 Verificando conexión SSH con GitHub..."
if ssh -T git@github.com 2>&1 | grep -q "successfully authenticated"; then
    echo "✅ Conexión SSH a GitHub OK."
else
    echo "⚠️ No se pudo verificar la conexión SSH a GitHub. Revisa tus claves."
fi
echo

# --- 6️⃣ RESUMEN FINAL ---
echo "🎉 Migración configurada correctamente (sin push)."
echo "Repos configurados:"
echo "  - Principal: ${MAIN_REMOTE}"
echo "  - SPP: ${SPP_REMOTE}"
echo "  - SPP-PORTS: ${SPP_PORTS_REMOTE}"
echo "  - ESP-IDF: ${ESP_IDF_REMOTE}"
echo
echo "💡 Siguientes pasos:"
echo "  1. Revisa los cambios en .gitmodules → 'git diff .gitmodules'"
echo "  2. Haz commit si estás conforme → 'git add .gitmodules && git commit -m \"Actualizar submódulos\"'"
echo "  3. Haz push manual cuando lo desees → 'git push origin main'"
echo
echo "✅ Script completado sin commits ni pushes automáticos."
