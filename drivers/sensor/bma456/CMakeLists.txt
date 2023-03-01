# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(bma456.c)
zephyr_library_sources(bma456_i2c.c)
zephyr_library_sources(bma456_spi.c)
zephyr_library_sources(bosch/bma4.c)
zephyr_library_sources_ifdef(CONFIG_BMA456_VARIANT_MM bosch/bma456mm.c)
zephyr_library_sources_ifdef(CONFIG_BMA456_VARIANT_H bosch/bma456h.c)
zephyr_library_sources_ifdef(CONFIG_BMA456_VARIANT_W bosch/bma456w.c)
zephyr_library_sources_ifdef(CONFIG_BMA456_VARIANT_AN bosch/bma456_an.c)
zephyr_library_sources_ifdef(CONFIG_BMA456_TRIGGER bma456_trigger.c)
