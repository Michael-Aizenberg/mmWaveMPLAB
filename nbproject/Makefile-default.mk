#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Main.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Main.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=main.c system.c roboclaw.c servo.c ultrasound.c motion.c lcd.c lcd_printf.c tof.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.o ${OBJECTDIR}/system.o ${OBJECTDIR}/roboclaw.o ${OBJECTDIR}/servo.o ${OBJECTDIR}/ultrasound.o ${OBJECTDIR}/motion.o ${OBJECTDIR}/lcd.o ${OBJECTDIR}/lcd_printf.o ${OBJECTDIR}/tof.o
POSSIBLE_DEPFILES=${OBJECTDIR}/main.o.d ${OBJECTDIR}/system.o.d ${OBJECTDIR}/roboclaw.o.d ${OBJECTDIR}/servo.o.d ${OBJECTDIR}/ultrasound.o.d ${OBJECTDIR}/motion.o.d ${OBJECTDIR}/lcd.o.d ${OBJECTDIR}/lcd_printf.o.d ${OBJECTDIR}/tof.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.o ${OBJECTDIR}/system.o ${OBJECTDIR}/roboclaw.o ${OBJECTDIR}/servo.o ${OBJECTDIR}/ultrasound.o ${OBJECTDIR}/motion.o ${OBJECTDIR}/lcd.o ${OBJECTDIR}/lcd_printf.o ${OBJECTDIR}/tof.o

# Source Files
SOURCEFILES=main.c system.c roboclaw.c servo.c ultrasound.c motion.c lcd.c lcd_printf.c tof.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Main.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ1024GB610
MP_LINKER_FILE_OPTION=,--script=p24FJ1024GB610.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/6124693dd0edfd46bfc07cd6d1f05b03d4972146 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/system.o: system.c  .generated_files/flags/default/321022583e8ea84dc618fab8df88ce695dd11a2c .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/system.o.d 
	@${RM} ${OBJECTDIR}/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  system.c  -o ${OBJECTDIR}/system.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/system.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/roboclaw.o: roboclaw.c  .generated_files/flags/default/9a2fff3bba05c25f2324d74db6a08844fff92452 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/roboclaw.o.d 
	@${RM} ${OBJECTDIR}/roboclaw.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  roboclaw.c  -o ${OBJECTDIR}/roboclaw.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/roboclaw.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/servo.o: servo.c  .generated_files/flags/default/9e9db507af9d6a489d552a4e6e8ae0440613bca6 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/servo.o.d 
	@${RM} ${OBJECTDIR}/servo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  servo.c  -o ${OBJECTDIR}/servo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/servo.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/ultrasound.o: ultrasound.c  .generated_files/flags/default/ddc14745217d1f20834a0bfc7adfc765d5ea4b2c .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ultrasound.o.d 
	@${RM} ${OBJECTDIR}/ultrasound.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ultrasound.c  -o ${OBJECTDIR}/ultrasound.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/ultrasound.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/motion.o: motion.c  .generated_files/flags/default/abd1525cb3a1a0635d0eae2ac5bfb3fffb9a7072 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/motion.o.d 
	@${RM} ${OBJECTDIR}/motion.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motion.c  -o ${OBJECTDIR}/motion.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motion.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/lcd.o: lcd.c  .generated_files/flags/default/46521d733707ee7a5780b2030df8647b9e4e3e19 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/lcd.o.d 
	@${RM} ${OBJECTDIR}/lcd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  lcd.c  -o ${OBJECTDIR}/lcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/lcd.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/lcd_printf.o: lcd_printf.c  .generated_files/flags/default/d30aca2d285da028c5649ce446ed0c0ebe627b15 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/lcd_printf.o.d 
	@${RM} ${OBJECTDIR}/lcd_printf.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  lcd_printf.c  -o ${OBJECTDIR}/lcd_printf.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/lcd_printf.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/tof.o: tof.c  .generated_files/flags/default/6b9b7e0d6d052e1914f2c579048fa877940fc71e .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/tof.o.d 
	@${RM} ${OBJECTDIR}/tof.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  tof.c  -o ${OBJECTDIR}/tof.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/tof.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
else
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/60dc35ab85876bcef8dfeed135a6d941f3eb6def .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/system.o: system.c  .generated_files/flags/default/5420618ab79a6a90e3d23b4d49e772b4b6fbd793 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/system.o.d 
	@${RM} ${OBJECTDIR}/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  system.c  -o ${OBJECTDIR}/system.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/system.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/roboclaw.o: roboclaw.c  .generated_files/flags/default/39302a407ade5dcae9a31696a65ce3764f9cc10b .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/roboclaw.o.d 
	@${RM} ${OBJECTDIR}/roboclaw.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  roboclaw.c  -o ${OBJECTDIR}/roboclaw.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/roboclaw.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/servo.o: servo.c  .generated_files/flags/default/66d1a22b165e0f1aeb7053912c2fb12e9f68af8c .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/servo.o.d 
	@${RM} ${OBJECTDIR}/servo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  servo.c  -o ${OBJECTDIR}/servo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/servo.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/ultrasound.o: ultrasound.c  .generated_files/flags/default/7b995447c2b2026512ee36d1a4cb2584db92a417 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ultrasound.o.d 
	@${RM} ${OBJECTDIR}/ultrasound.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ultrasound.c  -o ${OBJECTDIR}/ultrasound.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/ultrasound.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/motion.o: motion.c  .generated_files/flags/default/8268b9cea0953f9a2692278510d6e5d1fe3314bb .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/motion.o.d 
	@${RM} ${OBJECTDIR}/motion.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motion.c  -o ${OBJECTDIR}/motion.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motion.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/lcd.o: lcd.c  .generated_files/flags/default/25c683f0b56b32a66fa0350624539201d33411bc .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/lcd.o.d 
	@${RM} ${OBJECTDIR}/lcd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  lcd.c  -o ${OBJECTDIR}/lcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/lcd.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/lcd_printf.o: lcd_printf.c  .generated_files/flags/default/910e8e8ecbce50354a42d4c400328926523f9d32 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/lcd_printf.o.d 
	@${RM} ${OBJECTDIR}/lcd_printf.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  lcd_printf.c  -o ${OBJECTDIR}/lcd_printf.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/lcd_printf.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/tof.o: tof.c  .generated_files/flags/default/d7ba8373a58e1c0063d384033db456e68f952102 .generated_files/flags/default/bb9c332fb0c2e171c9cb4f01b7667c80b6e23b10
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/tof.o.d 
	@${RM} ${OBJECTDIR}/tof.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  tof.c  -o ${OBJECTDIR}/tof.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/tof.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Main.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Main.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)      -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Main.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Main.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Main.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
