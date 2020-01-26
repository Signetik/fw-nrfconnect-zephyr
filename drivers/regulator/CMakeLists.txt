# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_REGULATOR regulator_common.c)
zephyr_library_sources_ifdef(CONFIG_REGULATOR_GENERIC regulator_generic.c)
