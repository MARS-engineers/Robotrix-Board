#!/usr/bin/env python3
import os
import sys
import json

def BuildFlashMenu(name, chip, flashsize, fssizelist):
    if chip == "rp2350":
        delta = 8192
    elif chip == "rp2040":
        delta = 4096
    for fssize in fssizelist:
        if fssize == 0:
            fssizename = "no FS"
        elif fssize < 1024 * 1024:
            fssizename = "Sketch: %dKB, FS: %dKB" % ((flashsize - fssize) / 1024, fssize / 1024)
        else:
            fssizename = "Sketch: %dMB, FS: %dMB" % ((flashsize - fssize) / (1024 * 1024), fssize / (1024 * 1024))
        mn="%d_%d" % (flashsize, fssize)
        print("%s.menu.flash.%s=%dMB (%s)" % (name, mn, flashsize / (1024 * 1024), fssizename))
        print("%s.menu.flash.%s.upload.maximum_size=%d" % (name, mn, flashsize - delta - fssize))
        print("%s.menu.flash.%s.build.flash_total=%d" % (name, mn, flashsize))
        print("%s.menu.flash.%s.build.flash_length=%d" % (name, mn, flashsize - delta - fssize))
        print("%s.menu.flash.%s.build.eeprom_start=%d" % (name, mn, int("0x10000000",0) + flashsize - delta))
        print("%s.menu.flash.%s.build.fs_start=%d" % (name, mn, int("0x10000000",0) + flashsize - delta - fssize))
        print("%s.menu.flash.%s.build.fs_end=%d" % (name, mn, int("0x10000000",0) + flashsize - delta))

def BuildDebugPort(name):
    print("%s.menu.dbgport.Disabled=Disabled" % (name))
    print("%s.menu.dbgport.Disabled.build.debug_port=" % (name))
    for p in ["Serial", "Serial1", "Serial2", "SerialSemi"]:
        print("%s.menu.dbgport.%s=%s" % (name, p, p))
        print("%s.menu.dbgport.%s.build.debug_port=-DDEBUG_RP2040_PORT=%s" % (name, p, p))

def BuildDebugLevel(name):
    for l in [ ("None", ""), ("Core", "-DDEBUG_RP2040_CORE"), ("SPI", "-DDEBUG_RP2040_SPI"), ("Wire", "-DDEBUG_RP2040_WIRE"), ("Bluetooth", "-DDEBUG_RP2040_BLUETOOTH"),
               ("LWIP", "-DLWIP_DEBUG=1"), ("All", "-DDEBUG_RP2040_WIRE -DDEBUG_RP2040_SPI -DDEBUG_RP2040_CORE -DDEBUG_RP2040_BLUETOOTH -DLWIP_DEBUG=1"), ("NDEBUG", "-DNDEBUG") ]:
        print("%s.menu.dbglvl.%s=%s" % (name, l[0], l[0]))
        print("%s.menu.dbglvl.%s.build.debug_level=%s" % (name, l[0], l[1]))

def BuildFreq(name, defmhz):
    out = 0
    for f in [ defmhz, 50, 100, 120, 125, 128, 133, 150, 176, 200, 225, 240, 250, 276, 300]:
        warn = ""
        if f > defmhz: warn = " (Overclock)"
        if (out == 1) and (f == defmhz):
            continue
        out = 1
        print("%s.menu.freq.%s=%s MHz%s" % (name, f, f, warn))
        print("%s.menu.freq.%s.build.f_cpu=%dL" % (name, f, f * 1000000))

def BuildArch(name):
    # Cortex M-33
    print("%s.menu.arch.arm=ARM" % (name))
    print("%s.menu.arch.arm.build.chip=%s" % (name, "rp2350"))
    print("%s.menu.arch.arm.build.toolchain=arm-none-eabi" % (name))
    print("%s.menu.arch.arm.build.toolchainpkg=pqt-gcc" % (name))
    print("%s.menu.arch.arm.build.toolchainopts=-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -mcmse" % (name))
    print("%s.menu.arch.arm.build.uf2family=--family rp2350-arm-s --abs-block" % (name))
    print("%s.menu.arch.arm.build.mcu=cortex-m33" % (name))

    # RISC-V Hazard3
    print("%s.menu.arch.riscv=RISC-V" % (name))
    print("%s.menu.arch.riscv.build.chip=%s" % (name, "rp2350-riscv"))
    print("%s.menu.arch.riscv.build.toolchain=riscv32-unknown-elf" % (name))
    print("%s.menu.arch.riscv.build.toolchainpkg=pqt-gcc-riscv" % (name))
    print("%s.menu.arch.riscv.build.toolchainopts=-march=rv32imac_zicsr_zifencei_zba_zbb_zbs_zbkb -mabi=ilp32" % (name))
    print("%s.menu.arch.riscv.build.uf2family=--family rp2350-riscv --abs-block" % (name))
    print("%s.menu.arch.riscv.build.mcu=rv32imac" % (name))

def BuildPSRAM(name):
    for s in [ 0, 2, 4, 8]:
        print("%s.menu.psram.%dmb=%dMByte PSRAM" % (name, s, s))
        print("%s.menu.psram.%dmb.build.psram_length=0x%d00000" % (name, s, s))

def BuildPSRAMCS(name):
    print("%s.menu.psramcs.GPIOnone=None" % (name))
    print("%s.menu.psramcs.GPIOnone.build.psram_cs=" % (name))
    for s in range(0, 48):
        print("%s.menu.psramcs.GPIO%d=GPIO %d" % (name, s, s))
        print("%s.menu.psramcs.GPIO%d.build.psram_cs=-DRP2350_PSRAM_CS=%d" % (name, s, s))

def BuildPSRAMFreq(name):
    for s in [ 109, 133 ]:
        print("%s.menu.psramfreq.freq%d=%d MHz" % (name, s, s))
        print("%s.menu.psramfreq.freq%d.build.psram_freq=-DRP2350_PSRAM_MAX_SCK_HZ=%d" % (name, s, s * 1000000))

def BuildRP2350Variant(name):
    for l in [ ("RP2350A", "-D__PICO_RP2350A=1"), ("RP2530B", "-D__PICO_RP2350A=0") ]:
        print("%s.menu.variantchip.%s=%s" % (name, l[0], l[0]))
        print("%s.menu.variantchip.%s.build.variantdefines=%s" % (name, l[0], l[1]))

def BuildOptimize(name):
    for l in [ ("Small", "Small", "-Os", " (standard)"), ("Optimize", "Optimize", "-O", ""), ("Optimize2", "Optimize More", "-O2", ""),
               ("Optimize3", "Optimize Even More", "-O3", ""), ("Fast", "Fast", "-Ofast", " (maybe slower)"), ("Debug", "Debug", "-Og", ""),
               ("Disabled", "Disabled", "-O0", "") ]:
        print("%s.menu.opt.%s=%s (%s)%s" % (name, l[0], l[1], l[2], l[3]))
        print("%s.menu.opt.%s.build.flags.optimize=%s" % (name, l[0], l[2]))

def BuildOS(name):
    print("%s.menu.os.none=None" % (name))
    print("%s.menu.os.none.build.os=" % (name))
    print("%s.menu.os.freertos=FreeRTOS SMP" % (name))
    print("%s.menu.os.freertos.build.os=-D__FREERTOS" % (name))

def BuildProfile(name):
    print("%s.menu.profile.Disabled=Disabled" % (name))
    print("%s.menu.profile.Disabled.build.flags.profile=" % (name))
    print("%s.menu.profile.Enabled=Enabled" % (name))
    print("%s.menu.profile.Enabled.build.flags.profile=-pg -D__PROFILE" % (name))

def BuildRTTI(name):
    print("%s.menu.rtti.Disabled=Disabled" % (name))
    print("%s.menu.rtti.Disabled.build.flags.rtti=-fno-rtti" % (name))
    print("%s.menu.rtti.Enabled=Enabled" % (name))
    print("%s.menu.rtti.Enabled.build.flags.rtti=" % (name))

def BuildStackProtect(name):
    print("%s.menu.stackprotect.Disabled=Disabled" % (name))
    print("%s.menu.stackprotect.Disabled.build.flags.stackprotect=" % (name))
    print("%s.menu.stackprotect.Enabled=Enabled" % (name))
    print("%s.menu.stackprotect.Enabled.build.flags.stackprotect=-fstack-protector-all" % (name))

def BuildExceptions(name):
    print("%s.menu.exceptions.Disabled=Disabled" % (name))
    print("%s.menu.exceptions.Disabled.build.flags.exceptions=-fno-exceptions" % (name))
    print("%s.menu.exceptions.Disabled.build.flags.libstdcpp=-lstdc++" % (name))
    print("%s.menu.exceptions.Enabled=Enabled" % (name))
    print("%s.menu.exceptions.Enabled.build.flags.exceptions=-fexceptions" % (name))
    print("%s.menu.exceptions.Enabled.build.flags.libstdcpp=-lstdc++-exc" % (name))

def BuildBoot(name):
    for l in [ ("Generic SPI /2", "boot2_generic_03h_2_padded_checksum"),  ("Generic SPI /4", "boot2_generic_03h_4_padded_checksum"),
            ("IS25LP080 QSPI /2", "boot2_is25lp080_2_padded_checksum"), ("IS25LP080 QSPI /4", "boot2_is25lp080_4_padded_checksum"),
            ("W25Q080 QSPI /2", "boot2_w25q080_2_padded_checksum"), ("W25Q080 QSPI /4", "boot2_w25q080_4_padded_checksum"),
            ("W25X10CL QSPI /2", "boot2_w25x10cl_2_padded_checksum"), ("W25X10CL QSPI /4", "boot2_w25x10cl_4_padded_checksum"),
            ("W25Q64JV QSPI /4", "boot2_w25q64jv_4_padded_checksum"), ("W25Q16JVxQ QSPI /4", "boot2_w25q16jvxq_4_padded_checksum"),
            ("W25Q128JV QSPI /4", "boot2_w25q128jvxq_4_padded_checksum")]:
        print("%s.menu.boot2.%s=%s" % (name, l[1], l[0]))
        print("%s.menu.boot2.%s.build.boot2=%s" % (name, l[1], l[1]))

# Abbreviated Boot Stage 2 menu for some W25Q-equipped Adafruit boards.
# In extreme overclock situations, these may require QSPI /4 to work.
def BuildBootW25Q(name):
    for l in [ ("W25Q080 QSPI /2", "boot2_w25q080_2_padded_checksum"), ("W25Q080 QSPI /4", "boot2_w25q080_4_padded_checksum")]:
        print("%s.menu.boot2.%s=%s" % (name, l[1], l[0]))
        print("%s.menu.boot2.%s.build.boot2=%s" % (name, l[1], l[1]))

def BuildUSBStack(name):
    print("%s.menu.usbstack.picosdk=Pico SDK" % (name))
    print('%s.menu.usbstack.picosdk.build.usbstack_flags=' % (name))
    print("%s.menu.usbstack.tinyusb=Adafruit TinyUSB" % (name))
    print('%s.menu.usbstack.tinyusb.build.usbstack_flags=-DUSE_TINYUSB "-I{runtime.platform.path}/libraries/Adafruit_TinyUSB_Arduino/src/arduino"' % (name))
    print("%s.menu.usbstack.tinyusb_host=Adafruit TinyUSB Host (native)" % (name))
    print('%s.menu.usbstack.tinyusb_host.build.usbstack_flags=-DUSE_TINYUSB -DUSE_TINYUSB_HOST "-I{runtime.platform.path}/libraries/Adafruit_TinyUSB_Arduino/src/arduino"' % (name))
    print("%s.menu.usbstack.nousb=No USB" % (name))
    print('%s.menu.usbstack.nousb.build.usbstack_flags="-DNO_USB -DDISABLE_USB_SERIAL -I{runtime.platform.path}/tools/libpico"' % (name))

def BuildCountry(name):
    countries = [ "Worldwide", "Australia", "Austria", "Belgium", "Brazil", "Canada", "Chile", "China", "Colombia", "Czech Republic",
                  "Denmark", "Estonia", "Finland", "France", "Germany", "Greece", "Hong Kong", "Hungary", "Iceland", "India", "Israel",
                  "Italy", "Japan", "Kenya", "Latvia", "Liechtenstein", "Lithuania", "Luxembourg", "Malaysia", "Malta", "Mexico",
                  "Netherlands", "New Zealand", "Nigeria", "Norway", "Peru", "Philippines", "Poland", "Portugal", "Singapore", "Slovakia",
                  "Slovenia", "South Africa", "South Korea", "Spain", "Sweden", "Switzerland", "Taiwan", "Thailand", "Turkey", "UK", "USA"]
    for c in countries:
        sane = c.replace(" ", "_").upper()
        print("%s.menu.wificountry.%s=%s" % (name, sane.lower(), c))
        print("%s.menu.wificountry.%s.build.wificc=-DWIFICC=CYW43_COUNTRY_%s" % (name, sane.lower(), sane))

def BuildIPBTStack(name):
    print("%s.menu.ipbtstack.ipv4only=IPv4 Only" % (name))
    print('%s.menu.ipbtstack.ipv4only.build.libpicow=liblwip.a' % (name))
    print('%s.menu.ipbtstack.ipv4only.build.libpicowdefs=-DLWIP_IPV6=0 -DLWIP_IPV4=1' % (name))
    print("%s.menu.ipbtstack.ipv4ipv6=IPv4 + IPv6" % (name))
    print('%s.menu.ipbtstack.ipv4ipv6.build.libpicow=liblwip.a' % (name))
    print('%s.menu.ipbtstack.ipv4ipv6.build.libpicowdefs=-DLWIP_IPV6=1 -DLWIP_IPV4=1' % (name))
    print("%s.menu.ipbtstack.ipv4btcble=IPv4 + Bluetooth" % (name))
    print('%s.menu.ipbtstack.ipv4btcble.build.libpicow=liblwip-bt.a' % (name))
    print('%s.menu.ipbtstack.ipv4btcble.build.libpicowdefs=-DLWIP_IPV6=0 -DLWIP_IPV4=1 -DENABLE_CLASSIC=1 -DENABLE_BLE=1 -DCYW43_ENABLE_BLUETOOTH=1' % (name))
    print("%s.menu.ipbtstack.ipv4ipv6btcble=IPv4 + IPv6 + Bluetooth" % (name))
    print('%s.menu.ipbtstack.ipv4ipv6btcble.build.libpicow=liblwip-bt.a' % (name))
    print('%s.menu.ipbtstack.ipv4ipv6btcble.build.libpicowdefs=-DLWIP_IPV6=1 -DLWIP_IPV4=1 -DENABLE_CLASSIC=1 -DENABLE_BLE=1 -DCYW43_ENABLE_BLUETOOTH=1' % (name))
    print("%s.menu.ipbtstack.ipv4onlybig=IPv4 Only - 32K" % (name))
    print('%s.menu.ipbtstack.ipv4onlybig.build.libpicow=liblwip.a' % (name))
    print('%s.menu.ipbtstack.ipv4onlybig.build.libpicowdefs=-DLWIP_IPV6=0 -DLWIP_IPV4=1 -D__LWIP_MEMMULT=2' % (name))
    print("%s.menu.ipbtstack.ipv4ipv6big=IPv4 + IPv6 - 32K" % (name))
    print('%s.menu.ipbtstack.ipv4ipv6big.build.libpicow=liblwip.a' % (name))
    print('%s.menu.ipbtstack.ipv4ipv6big.build.libpicowdefs=-DLWIP_IPV6=1 -DLWIP_IPV4=1 -D__LWIP_MEMMULT=2' % (name))
    print("%s.menu.ipbtstack.ipv4btcblebig=IPv4 + Bluetooth - 32K" % (name))
    print('%s.menu.ipbtstack.ipv4btcblebig.build.libpicow=liblwip-bt.a' % (name))
    print('%s.menu.ipbtstack.ipv4btcblebig.build.libpicowdefs=-DLWIP_IPV6=0 -DLWIP_IPV4=1 -DENABLE_CLASSIC=1 -DENABLE_BLE=1 -DCYW43_ENABLE_BLUETOOTH=1 -D__LWIP_MEMMULT=2' % (name))
    print("%s.menu.ipbtstack.ipv4ipv6btcblebig=IPv4 + IPv6 + Bluetooth - 32K" % (name))
    print('%s.menu.ipbtstack.ipv4ipv6btcblebig.build.libpicow=liblwip-bt.a' % (name))
    print('%s.menu.ipbtstack.ipv4ipv6btcblebig.build.libpicowdefs=-DLWIP_IPV6=1 -DLWIP_IPV4=1 -DENABLE_CLASSIC=1 -DENABLE_BLE=1 -DCYW43_ENABLE_BLUETOOTH=1 -D__LWIP_MEMMULT=2' % (name))


def BuildUploadMethodMenu(name, ram):
    for a, b, c, d, e, f in [ ["default", "Default (UF2)", ram, "picoprobe_cmsis_dap.tcl", "uf2conv", "uf2conv-network"],
                              ["picotool", "Picotool", ram, "picoprobe.tcl", "picotool", None],
                              ["picoprobe_cmsis_dap", "Picoprobe/Debugprobe (CMSIS-DAP)", ram, "picoprobe_cmsis_dap.tcl", "picoprobe_cmsis_dap", None] ]:
        print("%s.menu.uploadmethod.%s=%s" % (name, a, b))
        print("%s.menu.uploadmethod.%s.build.ram_length=%dk" % (name, a, c))
        print("%s.menu.uploadmethod.%s.build.debugscript=%s" % (name, a, d))
        if a == "picotool":
            print("%s.menu.uploadmethod.%s.build.picodebugflags=-DENABLE_PICOTOOL_USB" % (name, a))
        print("%s.menu.uploadmethod.%s.upload.maximum_data_size=%d" % (name, a, c * 1024))
        print("%s.menu.uploadmethod.%s.upload.tool=%s" % (name, a, e))
        print("%s.menu.uploadmethod.%s.upload.tool.default=%s" % (name, a, e))
        if f != None:
            print("%s.menu.uploadmethod.%s.upload.tool.network=%s" % (name, a, f))

def BuildHeader(name, chip, chaintuple, chipoptions, vendor_name, product_name, vid, pid, pwr, boarddefine, variant, flashsize, psramsize, boot2, extra):
    prettyname = vendor_name + " " + product_name
    print()
    print("# -----------------------------------")
    print("# %s" % (prettyname))
    print("# -----------------------------------")
    print("%s.name=%s" % (name, prettyname))

    # USB Vendor ID / Product ID (VID/PID) pairs for board detection
    if isinstance(pid, list):
        # Explicitly specified list of PIDs (with the same VID)
        usb_pids = pid
    else:
        # When the RP2040 is used as a composite device, the PID is modified
        # (see cores/rp2040/RP2040USB.cpp) because Windows wants a different
        # VID:PID for different device configurations [citation needed?].
        # See https://github.com/earlephilhower/arduino-pico/issues/796
        #
        # TODO FIX: Some PIDs already have these bits set, and there's no
        # guarantee mangling PIDs this way won't collide with other devices.
        usb_pids = []
        for k_bit in [0, 0x8000]:
            for m_bit in [0, 0x4000]:
                for j_bit in [0, 0x0100]:
                    this_pid = "0x%04x" % (int(pid, 16) | k_bit | m_bit | j_bit)
                    if this_pid not in usb_pids:
                        usb_pids.append(this_pid)

    main_pid = usb_pids[0]

    # Old style VID/PID list for compatibility with older Arduino tools
    for i, pid in enumerate(usb_pids):
        print("%s.vid.%d=%s" % (name, i, vid))
        print("%s.pid.%d=%s" % (name, i, pid))

    # Since our platform.txt enables pluggable discovery, we are also required
    # to list VID/PID in this format
    for i, pid in enumerate(usb_pids):
        print("%s.upload_port.%d.vid=%s" % (name, i, vid))
        print("%s.upload_port.%d.pid=%s" % (name, i, pid))

    print("%s.build.usbvid=-DUSBD_VID=%s" % (name, vid))
    print("%s.build.usbpid=-DUSBD_PID=%s" % (name, main_pid))
    print("%s.build.usbpwr=-DUSBD_MAX_POWER_MA=%s" % (name, pwr))
    print("%s.build.board=%s" % (name, boarddefine))

    if chip == "rp2040":  # RP2350 has menu for this later on
        print("%s.build.mcu=cortex-m0plus" % (name))        
        print("%s.build.chip=%s" % (name, chip))
        print("%s.build.toolchain=%s" % (name, chaintuple))
        print("%s.build.toolchainpkg=%s" % (name, "pqt-gcc"))
        print("%s.build.toolchainopts=%s" % (name, chipoptions))
        print("%s.build.uf2family=%s" % (name, "--family rp2040"))
    print("%s.build.variant=%s" % (name, variant))
    print("%s.upload.maximum_size=%d" % (name, flashsize))
    print("%s.upload.wait_for_upload_port=true" % (name))
    print("%s.upload.erase_cmd=" % (name))
    print("%s.serial.disableDTR=false" % (name))
    print("%s.serial.disableRTS=false" % (name))
    print("%s.build.f_cpu=125000000" % (name))
    print("%s.build.led=" % (name))
    print("%s.build.core=rp2040" % (name))
    print("%s.build.ldscript=memmap_default.ld" % (name))
    print("%s.build.boot2=%s" % (name, boot2))
    print('%s.build.usb_manufacturer="%s"' % (name, vendor_name))
    print('%s.build.usb_product="%s"' % (name, product_name))
    if ((chip == "rp2350") or (chip == "rp2350-riscv")) and (name != "generic_rp2350"):
        print("%s.build.psram_length=0x%d00000" % (name, psramsize))

    if extra != None:
        m_extra = ''
        for m_item in extra:
            m_extra += '-D' + m_item + ' '
        print('%s.build.extra_flags=%s' % (name, m_extra.rstrip()))

def WriteWarning():
    print("# WARNING - DO NOT EDIT THIS FILE, IT IS MACHINE GENERATED")
    print("#           To change something here, edit tools/makeboards.py and")
    print("#           run 'python3 makeboards.py > ../boards.txt' to regenerate")
    print()

def BuildGlobalMenuList():
    print("menu.BoardModel=Model")
    print("menu.variantchip=Chip Variant")
    print("menu.flash=Flash Size")
    print("menu.psramcs=PSRAM CS")
    print("menu.psram=PSRAM Size")
    print("menu.psramfreq=PSRAM Speed")
    print("menu.freq=CPU Speed")
    print("menu.arch=CPU Architecture")
    print("menu.opt=Optimize")
    print("menu.os=Operating System")
    print("menu.profile=Profiling")
    print("menu.rtti=RTTI")
    print("menu.stackprotect=Stack Protector")
    print("menu.exceptions=C++ Exceptions")
    print("menu.dbgport=Debug Port")
    print("menu.dbglvl=Debug Level")
    print("menu.boot2=Boot Stage 2")
    print("menu.wificountry=WiFi Region")
    print("menu.usbstack=USB Stack")
    print("menu.espwifitype=ESP Wifi Type")
    print("menu.ipbtstack=IP/Bluetooth Stack")
    print("menu.uploadmethod=Upload Method")

def BuildWifiType(name):
    print("%s.menu.espwifitype.esp_at=ESP AT" % (name))
    print("%s.menu.espwifitype.esp_at.build.espwifitype=-DWIFIESPAT2" % (name))
    print("%s.menu.espwifitype.esp_hosted=ESP Hosted" % (name))
    print("%s.menu.espwifitype.esp_hosted.build.espwifitype=-DESPHOSTSPI=SPI1" % (name))

def MakeBoard(name, chip, vendor_name, product_name, vid, pid, pwr, boarddefine, flashsizemb, psramsize, boot2, extra = None, board_url = None):
    smallfs = [ 0, 64 * 1024, 128 * 1024, 256 * 1024, 512 * 1024 ]
    fssizelist = list(smallfs)
    for i in range(1, flashsizemb):
        fssizelist.append(i * 1024 * 1024)
    if chip == "rp2040":
        tup = "arm-none-eabi"
        opts = "-march=armv6-m -mcpu=cortex-m0plus -mthumb"
    elif chip == "rp2350":
        tup =  "arm-none-eabi"
        opts = "-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mfloat-abi=softfp -mcmse"
    elif chip == "rp2350-riscv":
        tup = "riscv32-unknown-elf"
        opts = "-march=rv32imac_zicsr_zifencei_zba_zbb_zbs_zbkb -mabi=ilp32"
    else:
        raise Exception("Unknown board type " + str(chip))
    BuildHeader(name, chip, tup, opts, vendor_name, product_name, vid, pid, pwr, boarddefine, name, flashsizemb * 1024 * 1024, psramsize, boot2, extra)
    if (name == "generic") or (name == "generic_rp2350") or (name == "vccgnd_yd_rp2040"):
        smfs =  [ 0, 64 * 1024, 128 * 1024, 256 * 1024, 512 * 1024 ]
        BuildFlashMenu(name, chip, 2*1024*1024, [*smallfs, 1024 * 1024])
        BuildFlashMenu(name, chip, 4*1024*1024, [0, 3*1024*1024, 2*1024*1024])
        BuildFlashMenu(name, chip, 8*1024*1024, [0, 7*1024*1024, 4*1024*1024, 2*1024*1024])
        BuildFlashMenu(name, chip, 16*1024*1024, [0, 15*1024*1024, 14*1024*1024, 12*1024*1024, 8*1024*1024, 4*1024*1024, 2*1024*1024])
    elif name == "pimoroni_tiny2040":
        BuildFlashMenu(name, chip, 2*1024*1024, [*smallfs, 1024 * 1024])
        BuildFlashMenu(name, chip, 8*1024*1024, [0, 7*1024*1024, 4*1024*1024, 2*1024*1024])
    elif name == "akana_r1":
        BuildFlashMenu(name, chip, 2*1024*1024, [*smallfs, 1024 * 1024])
        BuildFlashMenu(name, chip, 8*1024*1024, [0, 7*1024*1024, 4*1024*1024, 2*1024*1024])
        BuildFlashMenu(name, chip, 16*1024*1024, [0, 15*1024*1024, 14*1024*1024, 12*1024*1024, 8*1024*1024, 4*1024*1024, 2*1024*1024])
    elif name == "olimex_rp2040pico30":
        BuildFlashMenu(name, chip, 2*1024*1024, [*smallfs, 1024 * 1024])
        BuildFlashMenu(name, chip, 16*1024*1024, [0, 15*1024*1024, 14*1024*1024, 12*1024*1024, 8*1024*1024, 4*1024*1024, 2*1024*1024])
    elif (name == "challenger_2350_wifi6_ble5") or (name == "challenger_2040_wifi_ble"):
        BuildWifiType(name)
        BuildCountry(name)
        BuildFlashMenu(name, chip, 8*1024*1024, [0, 7*1024*1024, 4*1024*1024, 2*1024*1024])
        BuildFlashMenu(name, chip, 16*1024*1024, [0, 15*1024*1024, 14*1024*1024, 12*1024*1024, 8*1024*1024, 4*1024*1024, 2*1024*1024])
    elif name == "waveshare_rp2040_plus":
        BuildFlashMenu(name, chip, 4*1024*1024, [*smallfs, 1024*1024, 2*1024*1024, 3*1024*1024])
        BuildFlashMenu(name, chip, 16*1024*1024, [0, 15*1024*1024, 14*1024*1024, 12*1024*1024, 8*1024*1024, 4*1024*1024, 2*1024*1024])
    elif name == "waveshare_rp2350_plus":
        BuildFlashMenu(name, chip, 4*1024*1024, [*smallfs, 1024*1024, 2*1024*1024, 3*1024*1024])
        BuildFlashMenu(name, chip, 16*1024*1024, [0, 15*1024*1024, 14*1024*1024, 12*1024*1024, 8*1024*1024, 4*1024*1024, 2*1024*1024])
    elif name == "waveshare_rp2350b_plus_w":
        BuildFlashMenu(name, chip, 4*1024*1024, [*smallfs, 1024*1024, 2*1024*1024, 3*1024*1024])
        BuildFlashMenu(name, chip, 16*1024*1024, [0, 15*1024*1024, 14*1024*1024, 12*1024*1024, 8*1024*1024, 4*1024*1024, 2*1024*1024])
    else:
        BuildFlashMenu(name, chip, flashsizemb * 1024 * 1024, fssizelist)
    if (chip == "rp2350") or (chip == "rp2350-riscv"):
        BuildArch(name)
        BuildFreq(name, 150)
        if name == "generic_rp2350":
            BuildRP2350Variant(name)
            BuildPSRAMCS(name)
            BuildPSRAM(name)
            BuildPSRAMFreq(name)
        elif (name == "datanoisetv_picoadk_v2") or (name == "olimex_pico2bb48"):
            # Optional, user needs to solder themselves
            BuildPSRAM(name)
            BuildPSRAMFreq(name)
        elif (name == "adafruit_feather_rp2350_hstx") or (name == "adafruit_metro_rp2350"):
            # Optional, user needs to solder themselves
            BuildPSRAM(name)
    else:
        BuildFreq(name, 200)
    BuildOptimize(name)
    BuildOS(name)
    BuildProfile(name)
    BuildRTTI(name)
    BuildStackProtect(name)
    BuildExceptions(name)
    BuildDebugPort(name)
    BuildDebugLevel(name)
    BuildUSBStack(name)
    if name in ["rpipicow", "rpipico2w", "pimoroni_pico_plus_2w", "sparkfun_thingplusrp2350", "waveshare_rp2350b_plus_w"]:
        BuildCountry(name)
    BuildIPBTStack(name)
    if name == "generic":
        BuildBoot(name)
    elif name.startswith("adafruit") and "w25q080" in boot2:
        BuildBootW25Q(name)
    if chip == "rp2040":
        BuildUploadMethodMenu(name, 256)
    else:
        BuildUploadMethodMenu(name, 512)
    MakeBoardJSON(name, chip, vendor_name, product_name, vid, pid, pwr, boarddefine, flashsizemb, psramsize, boot2, extra, board_url)
    global pkgjson
    thisbrd = {}
    thisbrd['name'] = "%s %s" % (vendor_name, product_name)
    pkgjson['packages'][0]['platforms'][0]['boards'].append(thisbrd)

def MakeBoardJSON(name, chip, vendor_name, product_name, vid, pid, pwr, boarddefine, flashsizemb, psramsize, boot2, extra, board_url):
    # TODO FIX: Use the same expanded PID list as in BuildHeader above?
    if isinstance(pid, list):
        pid = pid[0]
    if extra != None:
        m_extra = ' '
        for m_item in extra:
            m_extra += '-D' + m_item + ' '
    else:
        m_extra = ''
    if chip == "rp2040":
        cpu = "cortex-m0plus"
        ramsize = 256
        jlink = "RP2040_M0_0"
        fcpu = "133000000L"
    elif chip == "rp2350":
        cpu = "cortex-m33"
        ramsize = 512
        jlink = "RP2350_M33_0"
        fcpu = "150000000L"
    elif chip == "rp2350-riscv":
        cpu = "riscv"
        ramsize = 512
        jlink = "RP2350_RV32_0"
        fcpu = "150000000L"
    j = {
    "build": {
        "arduino": {
            "earlephilhower": {
                "boot2_source": boot2 + ".S",
                "usb_vid": vid.upper().replace("X", "x"),
                "usb_pid": pid.upper().replace("X", "x"),
            }
        },
        "core": "earlephilhower",
        "cpu": cpu,
        "extra_flags": "-DARDUINO_" + boarddefine + " -DARDUINO_ARCH_RP2040 -DUSBD_MAX_POWER_MA=" + str(pwr) + " " + m_extra.rstrip(),
        "f_cpu": fcpu,
        "hwids": [
            [
                "0x2E8A",
                "0x00C0"
            ],
            [
                vid.upper().replace("X", "x"),
                pid.upper().replace("X", "x"),
            ]
        ],
        "mcu": chip,
        "variant": name,
    },
    "debug": {
        "jlink_device": jlink,
        "openocd_target": chip + ".cfg",
        "svd_path": chip + ".svd"
    },
    "frameworks": [
        "arduino",
        "picosdk"
    ],
    "name": product_name,
    "upload": {
        "maximum_ram_size": ramsize * 1024,
        "maximum_size": 1024 * 1024 * flashsizemb,
        "require_upload_port": True,
        "native_usb": True,
        "use_1200bps_touch": True,
        "wait_for_upload_port": False,
        "protocol": "picotool",
        "protocols": [
            "blackmagic",
            "cmsis-dap",
            "jlink",
            "raspberrypi-swd",
            "picotool",
            "picoprobe"
        ]
    },
    "url": board_url or 'https://www.raspberrypi.org/products/raspberry-pi-pico/',
    "vendor": vendor_name,
    }
    # add nonzero PSRAM sizes of known boards (can still be overwritten in platformio.ini)
    if (psramsize != 0) and (name != "generic_rp2350"):
        j["upload"]["psram_length"] = psramsize * 1024 * 1024

    jsondir = os.path.abspath(os.path.dirname(__file__)) + "/json"
    with open(jsondir + "/" + name + ".json", "w", newline='\n') as jout:
        json.dump(j, jout, indent=4)

pkgjson = json.load(open(os.path.abspath(os.path.dirname(__file__)) + '/../package/package_pico_index.template.json'))
pkgjson['packages'][0]['platforms'][0]['boards'] = []

sys.stdout = open(os.path.abspath(os.path.dirname(__file__)) + "/../boards.txt", "w", newline='\n')
WriteWarning()
BuildGlobalMenuList()

# Note to new board manufacturers:  Please add your board so that it sorts
# alphabetically starting with the company name and then the board name.
# Otherwise it is difficult to find a specific board in the menu.

# Generic
MakeBoard("generic", "rp2040", "Generic", "RP2040", "0x2e8a", "0xf00a", 250, "GENERIC_RP2040", 16, 0, "boot2_generic_03h_4_padded_checksum")
MakeBoard("generic_rp2350", "rp2350", "Generic", "RP2350", "0x2e8a", "0xf00f", 250, "GENERIC_RP2350", 16, 8, "none")

# MARS-Engineers
MakeBoard("robotrix_rover_one_v1", "rp2040", "MARS-Engineers", "Robotrix-Rover-One-V1", "0x2e8a", "0xf00f", 250, "GENERIC_RP2040", 16, 8, "none")
MakeBoard("robotrix_rover_one_v2", "rp2350", "MARS-Engineers", "Robotrix-Rover-One-V2", "0x2e8a", "0xf00f", 250, "GENERIC_RP2350", 16, 8, "none")


sys.stdout.close()
with open(os.path.abspath(os.path.dirname(__file__)) + '/../package/package_pico_index.template.json', 'w', newline='\n') as f:
    f.write(json.dumps(pkgjson, indent=3))
