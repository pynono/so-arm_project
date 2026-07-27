# Source before python vision scripts that use cv2.imshow (OpenCV Qt backend).
# Usage: source scripts/qt_opencv_env.sh
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
_VENV="${VIRTUAL_ENV:-/home/rookie/D053/.venv}"
_FONTS="$_VENV/lib/python3.13/site-packages/cv2/qt/fonts"
if [ -d "$_FONTS" ] || [ -L "$_FONTS" ]; then
  export QT_QPA_FONTDIR="${QT_QPA_FONTDIR:-$(readlink -f "$_FONTS" 2>/dev/null || echo "$_FONTS")}"
fi
unset _VENV _FONTS
