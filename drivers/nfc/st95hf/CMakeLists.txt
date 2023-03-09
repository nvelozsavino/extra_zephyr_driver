# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(st95hf.c)
zephyr_library_sources(st95hf_functions.c)
zephyr_library_sources(iso14443a.c)
zephyr_library_sources(iso14443b.c)
zephyr_library_sources(iso18092.c)
# zephyr_library_sources(st95hf_spi.c)
zephyr_library_sources_ifdef(CONFIG_ST95HF_TRIGGER st95hf_trigger.c)
