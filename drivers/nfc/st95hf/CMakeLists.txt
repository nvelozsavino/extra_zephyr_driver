# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(st95hf.c)
# zephyr_library_sources(st95hf_spi.c)
zephyr_library_sources_ifdef(CONFIG_ST95HF_TRIGGER st95hf_trigger.c)
