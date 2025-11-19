# ============================================================================
# Makefile for MS51 Voltmeter (SDCC Compiler)
# ============================================================================
# Project: Digital Voltmeter for Nuvoton MS51FB9AE
# Compiler: SDCC (Small Device C Compiler)
# Date: 2025-11-19
# ============================================================================

# Compiler and tools
CC = sdcc
OBJCOPY = objcopy
PACKIHX = packihx

# Target MCU
MCU = mcs51

# Project name
PROJECT = voltmeter_sdcc

# Directories
SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Source files
SRC_FILES = $(SRC_DIR)/voltmeter_sdcc.c \
            $(SRC_DIR)/delay_sdcc.c

# Include directories
INCLUDES = -I$(INC_DIR)

# Compiler flags
CFLAGS = -m$(MCU)           \
         $(INCLUDES)         \
         --model-small       \
         --opt-code-size     \
         --stack-auto        \
         --xram-size 0       \
         --code-size 16384   \
         --iram-size 256     \
         --std-sdcc99        \
         --less-pedantic

# Optimization level (0-3, or -Os for size)
# -O2: Good balance between speed and size
# -O3: Maximum speed optimization
# --opt-code-size: Optimize for code size
OPT_LEVEL = --opt-code-size

# Linker flags
LDFLAGS = -m$(MCU)           \
          --model-small      \
          --out-fmt-ihx

# Output files
IHX_FILE = $(BUILD_DIR)/$(PROJECT).ihx
HEX_FILE = $(BUILD_DIR)/$(PROJECT).hex
MAP_FILE = $(BUILD_DIR)/$(PROJECT).map
MEM_FILE = $(BUILD_DIR)/$(PROJECT).mem
ASM_FILE = $(BUILD_DIR)/$(PROJECT).asm
LST_FILE = $(BUILD_DIR)/$(PROJECT).lst

# ============================================================================
# TARGETS
# ============================================================================

.PHONY: all clean flash size help dirs

# Default target
all: dirs $(HEX_FILE) size

# Create necessary directories
dirs:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(OBJ_DIR)

# Compile and link
$(IHX_FILE): $(SRC_FILES)
	@echo "=== Compiling SDCC Project: $(PROJECT) ==="
	$(CC) $(CFLAGS) $(OPT_LEVEL) $(LDFLAGS) -o $(IHX_FILE) $(SRC_FILES)
	@echo "=== Compilation Complete ==="

# Convert IHX to HEX format
$(HEX_FILE): $(IHX_FILE)
	@echo "=== Converting to Intel HEX format ==="
	$(PACKIHX) $(IHX_FILE) > $(HEX_FILE)
	@echo "=== HEX file created: $(HEX_FILE) ==="

# Display memory usage
size: $(IHX_FILE)
	@echo "========================================="
	@echo "        MEMORY USAGE REPORT"
	@echo "========================================="
	@if [ -f $(MEM_FILE) ]; then \
		cat $(MEM_FILE); \
	else \
		echo "Memory file not generated"; \
	fi
	@echo ""
	@echo "Output files:"
	@ls -lh $(BUILD_DIR)/*.ihx $(BUILD_DIR)/*.hex 2>/dev/null || true
	@echo "========================================="

# Clean build artifacts
clean:
	@echo "=== Cleaning build artifacts ==="
	rm -rf $(BUILD_DIR)
	rm -f *.asm *.lst *.rst *.sym *.rel *.lk *.map *.mem
	rm -f $(SRC_DIR)/*.asm $(SRC_DIR)/*.lst $(SRC_DIR)/*.rst $(SRC_DIR)/*.sym $(SRC_DIR)/*.rel
	@echo "=== Clean complete ==="

# Flash to target (requires stcgal or other programmer)
flash: $(HEX_FILE)
	@echo "=== Flashing to MS51FB9AE ==="
	@echo "NOTE: Adjust programmer command for your setup"
	@echo "Example: stcgal -p /dev/ttyUSB0 $(HEX_FILE)"
	@echo ""
	@echo "⚠️  WARNING: Make sure you have backup firmware!"
	@echo "⚠️  WARNING: Code protection is disabled in default build"
	@echo ""
	@read -p "Press Enter to continue with flashing (Ctrl+C to cancel)..." dummy
	# Uncomment and modify for your programmer:
	# stcgal -p /dev/ttyUSB0 $(HEX_FILE)
	@echo "Flash command not configured. Edit Makefile to add your programmer."

# Disassemble (if needed for debugging)
disasm: $(IHX_FILE)
	@echo "=== Creating disassembly ==="
	@if [ -f $(BUILD_DIR)/$(PROJECT).asm ]; then \
		cat $(BUILD_DIR)/$(PROJECT).asm > $(ASM_FILE); \
		echo "Disassembly saved to: $(ASM_FILE)"; \
	else \
		echo "Assembly file not found"; \
	fi

# Display help
help:
	@echo "========================================="
	@echo "  MS51 Voltmeter Makefile Help"
	@echo "========================================="
	@echo ""
	@echo "Available targets:"
	@echo "  make           - Build the project (default)"
	@echo "  make all       - Same as 'make'"
	@echo "  make clean     - Remove all build artifacts"
	@echo "  make size      - Display memory usage"
	@echo "  make flash     - Flash firmware to MS51 (configure first!)"
	@echo "  make disasm    - Create disassembly listing"
	@echo "  make help      - Display this help message"
	@echo ""
	@echo "Build configuration:"
	@echo "  MCU:      $(MCU)"
	@echo "  Model:    small"
	@echo "  Optimize: $(OPT_LEVEL)"
	@echo ""
	@echo "Output files:"
	@echo "  IHX: $(IHX_FILE)"
	@echo "  HEX: $(HEX_FILE)"
	@echo "  MAP: $(MAP_FILE)"
	@echo ""
	@echo "To modify build options, edit this Makefile."
	@echo "========================================="

# Build with verbose output (for debugging)
verbose: CFLAGS += -V
verbose: all

# Build with all warnings
warnings: CFLAGS += -Wall
warnings: all

# Build for debugging (no optimization)
debug: OPT_LEVEL = -O0
debug: CFLAGS += --debug
debug: all

# Build for production (maximum optimization)
production: OPT_LEVEL = -O3 --opt-code-size
production: all

# Quick syntax check (compile only, no link)
check:
	@echo "=== Syntax check only ==="
	$(CC) $(CFLAGS) -c $(SRC_FILES)
	@echo "=== Syntax OK ==="
	@rm -f *.rel *.asm *.lst *.rst *.sym

# ============================================================================
# NOTES
# ============================================================================
#
# SDCC Memory Models:
#   --model-small  : variables in internal RAM (fastest, recommended)
#   --model-medium : variables in paged external RAM
#   --model-large  : variables in external RAM
#
# Common Issues:
#   - "Stack overflow" warning: Reduce local variables or use static
#   - "Code size exceeded": Reduce features or optimize
#   - Linker errors: Check that all source files are listed
#
# For MS51 Programming:
#   - Use stcgal or Nuvoton ICP programmer
#   - Example: stcgal -p /dev/ttyUSB0 -P stc15 build/voltmeter_sdcc.hex
#
# ============================================================================
