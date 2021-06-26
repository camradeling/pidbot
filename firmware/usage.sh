#!/bin/bash
MAP_FILE=${1}
RAM_START=$(cat ${MAP_FILE} | grep "RAM " | awk '{print $2}')
RAM_LENGTH=$(cat ${MAP_FILE} | grep "RAM " | awk '{print $3}')
RAM_END=$(cat ${MAP_FILE} | grep "RAMEND" | awk '{print $1}')
RAM_USED=$((${RAM_END}-${RAM_START}))
RAM_LEFT=$((${RAM_LENGTH}-${RAM_USED}))
printf "RAM used: 0x%x, RAM left: 0x%x\n" ${RAM_USED} ${RAM_LEFT}

FLASH_START=$(cat ${MAP_FILE} | grep "FLASH " | awk '{print $2}')
FLASH_LENGTH=$(cat ${MAP_FILE} | grep "FLASH " | awk '{print $3}')
FLASH_END=$(cat ${MAP_FILE} | grep "FLASHEND " | awk '{print $1}')
INITDATA_END=$(cat ${MAP_FILE} | grep "INITDATAEND " | awk '{print $1}')
INITDATA_LENGTH=$((${INITDATA_END}-${RAM_START}))
FLASH_USED=$((${FLASH_END}-${FLASH_START}+${INITDATA_LENGTH}))
FLASH_LEFT=$((${FLASH_LENGTH}-${FLASH_USED}))
printf "FLASH used: 0x%x, FLASH left: 0x%x\n" ${FLASH_USED} ${FLASH_LEFT}