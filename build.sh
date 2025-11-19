#!/bin/bash
#
# Build Script for MS51 Voltmeter Firmware
# Supports both development and production builds
#

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SOURCE_FILE="voltmeter_production.c"
OUTPUT_DIR="build"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Print colored message
print_msg() {
    echo -e "${2}${1}${NC}"
}

# Print header
print_header() {
    echo ""
    echo "════════════════════════════════════════════════════"
    echo "  MS51 Voltmeter Firmware Build Script"
    echo "════════════════════════════════════════════════════"
    echo ""
}

# Show usage
show_usage() {
    echo "Usage: $0 [build_type] [version]"
    echo ""
    echo "Build Types:"
    echo "  debug       - Development build (no protection)"
    echo "  production  - Production build (with protection)"
    echo "  standard    - Standard build (EEPROM + Watchdog, no protection)"
    echo "  test        - Run automated tests"
    echo "  clean       - Clean build artifacts"
    echo "  status      - Show current build status"
    echo ""
    echo "Versions:"
    echo "  original    - Original voltmeter_production.c (default)"
    echo "  refactored  - New refactored version with enhanced error handling"
    echo ""
    echo "Examples:"
    echo "  $0 debug"
    echo "  $0 debug refactored"
    echo "  $0 production refactored"
    echo "  $0 standard refactored"
    echo "  $0 test"
    echo ""
}

# Create build directory
create_build_dir() {
    if [ ! -d "$OUTPUT_DIR" ]; then
        mkdir -p "$OUTPUT_DIR"
        print_msg "Created build directory: $OUTPUT_DIR" "$GREEN"
    fi
}

# Build debug version
build_debug() {
    print_header
    print_msg "Building DEBUG version..." "$BLUE"
    print_msg "Code protection: DISABLED" "$YELLOW"
    echo ""

    # Ensure PRODUCTION_BUILD is commented out
    if grep -q "^#define PRODUCTION_BUILD" "$SOURCE_FILE"; then
        print_msg "WARNING: PRODUCTION_BUILD is defined!" "$RED"
        print_msg "Commenting it out for debug build..." "$YELLOW"
        sed -i 's/^#define PRODUCTION_BUILD/\/\/ #define PRODUCTION_BUILD/' "$SOURCE_FILE"
    fi

    create_build_dir

    # Note: Actual compilation would use Keil C51 compiler
    # This is a placeholder for the build command
    print_msg "Build configuration:" "$GREEN"
    echo "  - Source: $SOURCE_FILE"
    echo "  - Mode: DEVELOPMENT"
    echo "  - Protection: OFF"
    echo "  - Debug: ON"
    echo ""

    print_msg "✓ Debug build prepared" "$GREEN"
    print_msg "Flash this version for testing and development" "$BLUE"
    echo ""
    print_msg "Next steps:" "$YELLOW"
    echo "  1. Open Keil µVision"
    echo "  2. Build → Rebuild All"
    echo "  3. Flash → Download"
    echo ""
}

# Build production version
build_production() {
    print_header
    print_msg "⚠️  WARNING: PRODUCTION BUILD ⚠️" "$RED"
    echo ""
    print_msg "This build will enable code protection!" "$YELLOW"
    print_msg "Device will be LOCKED after first power-on!" "$YELLOW"
    echo ""
    read -p "Have you completed ALL testing? (yes/no): " response

    if [ "$response" != "yes" ]; then
        print_msg "Build cancelled." "$YELLOW"
        exit 0
    fi

    echo ""
    print_msg "Building PRODUCTION version..." "$BLUE"
    print_msg "Code protection: ENABLED" "$RED"
    echo ""

    # Ensure PRODUCTION_BUILD is uncommented
    if ! grep -q "^#define PRODUCTION_BUILD" "$SOURCE_FILE"; then
        print_msg "Enabling PRODUCTION_BUILD..." "$YELLOW"
        sed -i 's/^\/\/ #define PRODUCTION_BUILD/#define PRODUCTION_BUILD/' "$SOURCE_FILE"
    fi

    create_build_dir

    # Create backup
    BACKUP_FILE="${OUTPUT_DIR}/voltmeter_production_${TIMESTAMP}.c.backup"
    cp "$SOURCE_FILE" "$BACKUP_FILE"
    print_msg "Backup created: $BACKUP_FILE" "$GREEN"

    print_msg "Build configuration:" "$GREEN"
    echo "  - Source: $SOURCE_FILE"
    echo "  - Mode: PRODUCTION"
    echo "  - Protection: ON"
    echo "  - Debug: OFF"
    echo "  - Timestamp: $TIMESTAMP"
    echo ""

    print_msg "✓ Production build prepared" "$GREEN"
    print_msg "⚠️  REMEMBER: Device will be LOCKED after first power-on!" "$RED"
    echo ""
    print_msg "Next steps:" "$YELLOW"
    echo "  1. Open Keil µVision"
    echo "  2. Build → Rebuild All"
    echo "  3. VERIFY: Check for production warnings"
    echo "  4. Flash → Download"
    echo "  5. Power cycle device"
    echo "  6. Observe '888' → '---' on display"
    echo "  7. Verify cannot read code back"
    echo ""
    print_msg "Documentation:" "$BLUE"
    echo "  See CODE_PROTECTION_GUIDE.md for details"
    echo ""
}

# Run tests
run_tests() {
    print_header
    print_msg "Running automated tests..." "$BLUE"
    echo ""

    if [ ! -f "test_simulation" ]; then
        print_msg "Compiling test suite..." "$YELLOW"
        gcc -o test_simulation test_simulation.c -lm
    fi

    print_msg "Executing tests..." "$GREEN"
    ./test_simulation

    if [ $? -eq 0 ]; then
        echo ""
        print_msg "✓ All tests passed!" "$GREEN"
        print_msg "Firmware is ready for production build" "$BLUE"
    else
        echo ""
        print_msg "✗ Some tests failed!" "$RED"
        print_msg "Fix issues before building production version" "$YELLOW"
        exit 1
    fi
    echo ""
}

# Clean build artifacts
clean_build() {
    print_header
    print_msg "Cleaning build artifacts..." "$YELLOW"
    echo ""

    if [ -d "$OUTPUT_DIR" ]; then
        rm -rf "$OUTPUT_DIR"
        print_msg "✓ Removed $OUTPUT_DIR" "$GREEN"
    fi

    if [ -f "test_simulation" ]; then
        rm -f test_simulation
        print_msg "✓ Removed test_simulation" "$GREEN"
    fi

    # Reset source to debug mode
    if grep -q "^#define PRODUCTION_BUILD" "$SOURCE_FILE"; then
        sed -i 's/^#define PRODUCTION_BUILD/\/\/ #define PRODUCTION_BUILD/' "$SOURCE_FILE"
        print_msg "✓ Reset to debug mode" "$GREEN"
    fi

    echo ""
    print_msg "Clean complete" "$GREEN"
    echo ""
}

# Show build status
show_status() {
    print_header
    print_msg "Current Build Status:" "$BLUE"
    echo ""

    if grep -q "^#define PRODUCTION_BUILD" "$SOURCE_FILE"; then
        print_msg "Mode: PRODUCTION" "$RED"
        print_msg "Protection: ENABLED" "$RED"
    else
        print_msg "Mode: DEVELOPMENT" "$GREEN"
        print_msg "Protection: DISABLED" "$GREEN"
    fi

    echo ""

    if [ -f "voltmeter.c" ]; then
        if grep -q "#define VOLTAGE_SCALE_NUM     100UL" voltmeter.c; then
            print_msg "Voltage calculation: FIXED (100UL) ✓" "$GREEN"
        else
            print_msg "Voltage calculation: CHECK NEEDED" "$YELLOW"
        fi
    fi

    echo ""

    if [ -f "test_simulation" ]; then
        print_msg "Test executable: Available" "$GREEN"
    else
        print_msg "Test executable: Not compiled" "$YELLOW"
    fi

    echo ""

    if [ -d "$OUTPUT_DIR" ]; then
        FILES=$(ls -1 "$OUTPUT_DIR" 2>/dev/null | wc -l)
        print_msg "Build directory: $FILES file(s)" "$BLUE"
    else
        print_msg "Build directory: Not created" "$YELLOW"
    fi

    echo ""
}

# Build refactored version
build_refactored() {
    local build_mode=$1
    print_header
    print_msg "Building REFACTORED version - $build_mode mode..." "$BLUE"
    echo ""

    local source="voltmeter_refactored.c"

    if [ ! -f "$source" ]; then
        print_msg "ERROR: $source not found!" "$RED"
        exit 1
    fi

    # Show which build mode will be active
    case "$build_mode" in
        debug|development)
            print_msg "Configuration: BUILD_DEVELOPMENT" "$GREEN"
            print_msg "  - Code protection: OFF" "$GREEN"
            print_msg "  - EEPROM: OFF" "$GREEN"
            print_msg "  - Watchdog: OFF" "$GREEN"
            print_msg "  - Error display: ON" "$GREEN"
            print_msg "  - Filtering: Fast (50%/50%)" "$GREEN"
            print_msg "  - Update rate: 150ms" "$GREEN"
            ;;
        standard)
            print_msg "Configuration: BUILD_STANDARD" "$YELLOW"
            print_msg "  - Code protection: OFF" "$YELLOW"
            print_msg "  - EEPROM: ON" "$GREEN"
            print_msg "  - Watchdog: ON" "$GREEN"
            print_msg "  - Error display: OFF" "$YELLOW"
            print_msg "  - Filtering: Balanced (50%/50%)" "$GREEN"
            print_msg "  - Update rate: 150ms" "$GREEN"
            ;;
        production)
            print_msg "Configuration: BUILD_PRODUCTION" "$RED"
            print_msg "  - Code protection: ON" "$RED"
            print_msg "  - EEPROM: ON" "$GREEN"
            print_msg "  - Watchdog: ON" "$GREEN"
            print_msg "  - Voltage multiplier: ON (x2.78)" "$GREEN"
            print_msg "  - Filtering: Maximum stability (12.5%/87.5%)" "$GREEN"
            print_msg "  - Update rate: 200ms" "$GREEN"

            echo ""
            print_msg "⚠️  WARNING: Code protection will be enabled!" "$RED"
            read -p "Continue? (yes/no): " response
            if [ "$response" != "yes" ]; then
                print_msg "Build cancelled." "$YELLOW"
                exit 0
            fi
            ;;
    esac

    echo ""
    print_msg "✓ Refactored build configured" "$GREEN"
    print_msg "Edit $source to select build mode, then compile in Keil" "$BLUE"
    echo ""
    print_msg "Next steps:" "$YELLOW"
    echo "  1. Edit $source"
    echo "  2. Uncomment the appropriate BUILD_xxx define"
    echo "  3. Open Keil µVision"
    echo "  4. Set source to: $source"
    echo "  5. Build → Rebuild All"
    echo "  6. Flash → Download"
    echo ""
}

# Main script
VERSION="${2:-original}"

case "${1:-help}" in
    debug|dev|development)
        if [ "$VERSION" = "refactored" ]; then
            build_refactored "debug"
        else
            build_debug
        fi
        ;;
    standard)
        if [ "$VERSION" = "refactored" ]; then
            build_refactored "standard"
        else
            print_msg "Standard build only available for refactored version" "$YELLOW"
            print_msg "Try: $0 standard refactored" "$BLUE"
            exit 1
        fi
        ;;
    prod|production)
        if [ "$VERSION" = "refactored" ]; then
            build_refactored "production"
        else
            build_production
        fi
        ;;
    test|tests)
        run_tests
        ;;
    clean)
        clean_build
        ;;
    status)
        show_status
        ;;
    help|-h|--help)
        show_usage
        ;;
    *)
        print_msg "Unknown command: $1" "$RED"
        show_usage
        exit 1
        ;;
esac

exit 0
