# OpenART Plus entrypoint.

from openart_config import PLUS_CONFIG, set_active_config

set_active_config(PLUS_CONFIG)

# Importing openart_app starts the OpenMV/OpenART runtime loop.
import openart_app
