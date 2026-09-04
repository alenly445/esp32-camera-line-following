#!/bin/zsh
set -euo pipefail

export PATH="/opt/homebrew/bin:/Users/mengyangzu/.espressif/tools/ninja/1.12.1:/usr/bin:/bin:/usr/sbin:/sbin"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE CONDA_SHLVL
unset IDF_PATH IDF_PATH_OLD IDF_PYTHON_ENV_PATH ESP_IDF_VERSION
unset CMAKE_PREFIX_PATH CMAKE_ARGS LDFLAGS CPPFLAGS CFLAGS CXXFLAGS

source "/Users/mengyangzu/.espressif/v5.4.4/esp-idf/export.sh"
idf.py "$@"
