# OpenART Mini entrypoint.

from openart_config import MINI_CONFIG, set_active_config

set_active_config(MINI_CONFIG)

# Importing openart_app starts the OpenMV/OpenART runtime loop.
import openart_app
