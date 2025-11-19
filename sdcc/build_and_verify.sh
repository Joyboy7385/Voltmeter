#!/bin/bash
# ============================================================================
# Build and Verification Script for MS51 Voltmeter (SDCC)
# ============================================================================
# This script builds the project and performs comprehensive verification
# Date: 2025-11-19
# ============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Counters
CHECKS_PASSED=0
CHECKS_FAILED=0
WARNINGS=0

# Functions
print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
    ((CHECKS_PASSED++))
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
    ((CHECKS_FAILED++))
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
    ((WARNINGS++))
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# ============================================================================
# STEP 1: Pre-build Checks
# ============================================================================
print_header "STEP 1: PRE-BUILD CHECKS"

# Check if SDCC is installed
if command -v sdcc &> /dev/null; then
    SDCC_VERSION=$(sdcc --version | head -n1)
    print_success "SDCC is installed: $SDCC_VERSION"
else
    print_error "SDCC is not installed!"
    echo "Please install SDCC: sudo apt-get install sdcc"
    exit 1
fi

# Check if packihx is installed
if command -v packihx &> /dev/null; then
    print_success "packihx is installed"
else
    print_error "packihx is not installed!"
    echo "Please install packihx (usually comes with SDCC)"
    exit 1
fi

# Check if make is installed
if command -v make &> /dev/null; then
    print_success "make is installed"
else
    print_error "make is not installed!"
    echo "Please install make: sudo apt-get install build-essential"
    exit 1
fi

# Check directory structure
if [ -d "include" ] && [ -d "src" ]; then
    print_success "Directory structure is correct"
else
    print_error "Missing include/ or src/ directories"
    exit 1
fi

# Check required header files
for header in "ms51_sdcc.h" "sfr_macro_sdcc.h" "function_define_sdcc.h"; do
    if [ -f "include/$header" ]; then
        print_success "Found header: $header"
    else
        print_error "Missing header: $header"
        exit 1
    fi
done

# Check required source files
for source in "voltmeter_sdcc.c" "delay_sdcc.c"; do
    if [ -f "src/$source" ]; then
        print_success "Found source: $source"
    else
        print_error "Missing source: $source"
        exit 1
    fi
done

echo ""

# ============================================================================
# STEP 2: Clean Build
# ============================================================================
print_header "STEP 2: CLEAN BUILD"

echo "Cleaning previous build..."
make clean > /dev/null 2>&1 || true
print_success "Clean complete"

echo ""

# ============================================================================
# STEP 3: Compilation
# ============================================================================
print_header "STEP 3: COMPILATION"

echo "Starting compilation..."
if make all 2>&1 | tee build.log; then
    print_success "Compilation successful"
else
    print_error "Compilation failed!"
    echo "Check build.log for details"
    exit 1
fi

echo ""

# ============================================================================
# STEP 4: Post-Build Verification
# ============================================================================
print_header "STEP 4: POST-BUILD VERIFICATION"

# Check if output files exist
if [ -f "build/voltmeter_sdcc.ihx" ]; then
    print_success "IHX file generated"
else
    print_error "IHX file missing!"
fi

if [ -f "build/voltmeter_sdcc.hex" ]; then
    print_success "HEX file generated"
    HEX_SIZE=$(wc -c < build/voltmeter_sdcc.hex)
    print_info "HEX file size: $HEX_SIZE bytes"
else
    print_error "HEX file missing!"
fi

# Check for warnings in build log
if grep -i "warning" build.log > /dev/null; then
    WARNING_COUNT=$(grep -c -i "warning" build.log)
    print_warning "Found $WARNING_COUNT compiler warnings"
    echo "  Review build.log for details"
else
    print_success "No compiler warnings"
fi

# Check for errors (should be none if we got here)
if grep -i "error" build.log > /dev/null; then
    ERROR_COUNT=$(grep -c -i "error" build.log)
    print_error "Found $ERROR_COUNT errors in log"
else
    print_success "No errors in build log"
fi

# Memory usage check
if [ -f "build/voltmeter_sdcc.mem" ]; then
    print_success "Memory report generated"
    print_info "Memory usage:"
    cat build/voltmeter_sdcc.mem | grep -E "CODE|DATA|IDATA|XDATA" | head -10 || true
else
    print_warning "Memory report not found"
fi

echo ""

# ============================================================================
# STEP 5: Code Analysis
# ============================================================================
print_header "STEP 5: CODE ANALYSIS"

# Check code size
if [ -f "build/voltmeter_sdcc.mem" ]; then
    CODE_SIZE=$(grep -i "^CODE" build/voltmeter_sdcc.mem | awk '{print $NF}' || echo "unknown")
    print_info "Code size: $CODE_SIZE bytes"

    # MS51FB9AE has 16KB flash
    MAX_CODE=16384
    if [ "$CODE_SIZE" != "unknown" ]; then
        if [ "$CODE_SIZE" -lt "$MAX_CODE" ]; then
            print_success "Code fits in 16KB flash"
        else
            print_error "Code exceeds 16KB flash limit!"
        fi
    fi
fi

# Check for common issues in source
print_info "Checking source code..."

# Check for magic numbers
if grep -rn "0x[0-9A-Fa-f]\{2,\}" src/ | grep -v "define" | grep -v "//" > /dev/null; then
    print_warning "Found potential magic numbers in source"
else
    print_success "No obvious magic numbers"
fi

# Check for TODO comments
if grep -rn "TODO\|FIXME\|XXX" src/ > /dev/null; then
    TODO_COUNT=$(grep -rc "TODO\|FIXME\|XXX" src/ | awk -F: '{sum+=$2} END {print sum}')
    print_warning "Found $TODO_COUNT TODO/FIXME comments"
else
    print_success "No TODO/FIXME comments"
fi

echo ""

# ============================================================================
# STEP 6: Final Summary
# ============================================================================
print_header "FINAL SUMMARY"

echo -e "${GREEN}Checks passed: $CHECKS_PASSED${NC}"
echo -e "${RED}Checks failed: $CHECKS_FAILED${NC}"
echo -e "${YELLOW}Warnings: $WARNINGS${NC}"

echo ""

if [ $CHECKS_FAILED -eq 0 ]; then
    print_success "BUILD VERIFICATION SUCCESSFUL!"
    echo ""
    echo "Next steps:"
    echo "  1. Review build.log for any warnings"
    echo "  2. Test hex file in simulator (if available)"
    echo "  3. Flash to hardware using programmer"
    echo "  4. Follow hardware testing checklist"
    echo ""
    echo "HEX file ready: build/voltmeter_sdcc.hex"
    exit 0
else
    print_error "BUILD VERIFICATION FAILED!"
    echo ""
    echo "Please fix the errors above before proceeding."
    exit 1
fi
