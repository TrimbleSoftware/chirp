# Copyright 2025 Fred Trimble <chirpdriver@gmail.com>
# CHIRP driver for TIDRADIO TD-M11-221 radios
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from chirp import (
    bitwise,
    chirp_common,
    directory,
    errors,
    memmap
)

from chirp.settings import (
    RadioSetting,
    RadioSettingGroup,
    RadioSettings,
    RadioSettingValueList,
    RadioSettingValueString,
    MemSetting
    # InvalidValueError
)

import logging
import struct
from textwrap import dedent

LOG = logging.getLogger(__name__)

MEM_FORMAT = """
// memmory channels: 16 bytes each * 22 channels = 352 bytes
struct {
  lbcd rxfreq[4]; //
  lbcd txfreq[4]; //
  lbcd rxtone[2]; //
  lbcd txtone[2]; //
  u8 unknown0:2,
     jumpfreq:1, //
     scan:1, //
     txpower:1, //
     narrow:1, //
     bcl:2; //
  u8 unknown1:3,
     compand:1, //
     scramble:4; //
  u8 unknown2;
  u8 unknown3;
} memory[%d];

// settings:
#seekto 0x0300;
struct {
  u8 unknown0:4,
     squelch:4; //
  u8 unused1:6, //
     voice:2; // voice prompts, 0 Off, 1 Chinese, 2 English
  u8 channel; // selected channel on power up
  u8 unused2:4,//
     vox:4; //
  u8 unused3:2, //
     tot:6; //
  u8 unknown2;
  u8 unused4:4, //
     sleep:4; //
  u8 firmware[4];
  u8 unused5:4, //
     sidekey1:4; //
  u8 unknown4;
  u8 unused6:4, //
     sidekey2:4; //
  u8 unknown5[2];
  u8 freqrange[4];
  u8 unknown6[12];
  u8 password[6]; //
  u8 unknown7[26];
} settings;
"""

CMD_ACK = b'A'
TIMEOUT = 1  # serial timeout in seconds
TXPOWER_HIGH = 0x01
TXPOWER_LOW = 0x00


def get_default_features(self):
    rf = chirp_common.RadioFeatures()
    rf.has_settings = True
    rf.has_bank = False
    rf.has_ctone = True
    rf.has_cross = True
    rf.has_rx_dtcs = True
    rf.has_tuning_step = False
    rf.can_odd_split = True
    rf.has_name = False
    rf.valid_name_length = 0
    rf.valid_characters = self._valid_chars
    rf.valid_skips = ["", "S"]
    rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
    rf.valid_cross_modes = ["Tone->Tone", "Tone->DTCS", "DTCS->Tone",
                            "->Tone", "->DTCS", "DTCS->", "DTCS->DTCS"]
    rf.valid_power_levels = self._power_levels
    rf.valid_duplexes = ["", "-", "+", "split", "off"]
    rf.valid_modes = ["FM", "NFM"]  # 25 kHz, 12.5 kHz.
    rf.valid_dtcs_codes = chirp_common.DTCS_CODES
    rf.memory_bounds = (1, self._upper)
    rf.valid_tuning_steps = [2.5, 5., 6.25, 8.33, 10., 12.5, 20., 25., 50.]
    rf.valid_bands = self.VALID_BANDS
    return rf


def _enter_programming_mode(radio):
    serial = radio.pipe
    serial.timeout = TIMEOUT

    exito = False
    for i in range(0, 5):
        serial.write(radio._magic)
        ack = serial.read(1)

        try:
            if ack == CMD_ACK:
                exito = True
                break
        except Exception:
            LOG.debug("Attempt #%s, failed, trying again" % i)
            pass

    # check if we had EXITO
    if exito is False:
        msg = "The radio did not accept program mode after five tries.\n"
        msg += "Check you interface cable and power cycle your radio."
        raise errors.RadioError(msg)

    exito = False
    try:
        # if we get an ACK here the radio programming function
        # is not password protected
        serial.write(b"\xff\xff\xff\xff\xff\xff")
        ack = serial.read(1)

        if ack == CMD_ACK:
            exito = True
        else:
            msg = 'The radio Programming function is '\
                'password protected. Use the factory CPS to '\
                'remove the password to continue.'
            raise ValueError(msg)

    except ValueError as ex:
        raise errors.RadioError(ex)

    except Exception:
        raise errors.RadioError("Error communicating with radio")


def _exit_programming_mode(radio):
    serial = radio.pipe
    serial.timeout = TIMEOUT

    try:
        serial.write(b"E")
    except Exception:
        raise errors.RadioError("Radio refused to exit programming mode")


def _get_checksum(data, addr, len):
    checksum = 0
    for index in range(0, len):
        checksum += data[index + addr]
    return checksum & 0xff  # checksum is only low order byte


def _read_block(radio, block_addr, size):
    serial = radio.pipe
    serial.timeout = TIMEOUT

    cmd = struct.pack('<cHb', b'R', block_addr, size)
    expectedresponse = b''

    try:
        serial.write(cmd)
        response = serial.read(size)
        expectedresponse = serial.read(1)  # read 1 byte checksum
        checksum = _get_checksum(response, 0, size)
        if checksum != int.from_bytes(expectedresponse, 'big'):
            raise errors.RadioError("Error reading block %04x." % (block_addr))
        block_data = response

    except Exception:
        raise errors.RadioError("Failed to read block at %04x" % block_addr)

    return block_data


def _write_block(radio, block_addr, size):
    serial = radio.pipe
    serial.timeout = TIMEOUT

    cmd = struct.pack('<cH', b'W', block_addr)
    data = radio.get_mmap()[block_addr:block_addr + size]
    checksum = _get_checksum(data, 0, size)

    try:
        serial.write(cmd + data + int.to_bytes(checksum, 1, 'big'))
        ack = serial.read(1)
        if ack != CMD_ACK:
            raise Exception("No ACK")
    except Exception:
        raise errors.RadioError("Failed to send block "
                                "to radio at %04x" % block_addr)


def do_download(radio):
    LOG.debug("Downloading...")
    _enter_programming_mode(radio)

    data = b''

    status = chirp_common.Status()
    status.msg = "Cloning from radio"

    status.cur = 0
    status.max = radio._memsize

    for addr in range(0, radio._memsize, radio.BLOCK_SIZE):
        status.cur = addr + radio.BLOCK_SIZE
        radio.status_fn(status)
        radio.pipe.log('Sending request for %04x' % addr)
        block = _read_block(radio, addr, radio.BLOCK_SIZE)
        data += block

    return memmap.MemoryMapBytes(data)


def do_upload(radio):
    LOG.debug("Uploading...")
    status = chirp_common.Status()
    status.msg = "Uploading to radio"
    _enter_programming_mode(radio)

    status.cur = 0
    # calc actual upload size
    for r in radio._ranges:
        status.max += (r[1] - r[0]) + 1

    for start_addr, end_addr in radio._ranges:
        for addr in range(start_addr, end_addr, radio.BLOCK_SIZE_UP):
            status.cur = addr + radio.BLOCK_SIZE_UP
            radio.status_fn(status)
            radio.pipe.log('Sending address %04x' % addr)
            _write_block(radio, addr, radio.BLOCK_SIZE_UP)


@directory.register
class TDM11_22(chirp_common.CloneModeRadio):
    # ==========
    # Notice to developers:
    # The TD-M11 22 for USA FRS/GMRS support in this driver is currently based
    # on Firmware v0.9.4
    # ==========
    """TIDRADIO TD-M11 22"""
    VENDOR = "TIDRADIO"
    MODEL = "TD-M11"
    VARIANT = '22 FRS'  # USA FRS/GMRS
    BAUD_RATE = 9600
    BLOCK_SIZE = 0x10
    BLOCK_SIZE_UP = 0x10
    VALID_BANDS = [(136000000, 174000001), (200000000, 260000001),
                   (350000000, 390000001), (400000000, 520000001)]
    _power_levels = [
        chirp_common.PowerLevel("Low", watts=0.50),
        chirp_common.PowerLevel("High", watts=2.00)
    ]

    _upper = 22
    _mem_params = (_upper)
    _memsize = 0x0340  # Including calibration data
    _ranges = [
        (0x0000, 0x0160),  # 22ea  16 byte channels
        (0x0300, 0X033F)  # settings and password
    ]
    _magic = b"STD-M11-"
    _valid_chars = chirp_common.CHARSET_ALPHANUMERIC
    _steps = [5.0, 12.5, 25.0]
    # mem extra lists
    _bcl_list = ['Off', 'Carrier', 'QT/DTQ']
    _jumpfreq_list = ['Normal', 'Special']
    _compand_list = ['Off', 'On']
    _scramble_list = ['Off'] + ['Scramble %d' % x for x in range(1, 9)]
    # settings lists
    _voice_list = ['Off', 'Chinese', 'English']
    _tot_list = ['Off'] + ['%ds' % x for x in range(15, 315, 15)]
    _voxlevel_list = ['Off'] + ['%d' % x for x in range(1, 10)]
    _squelchlevel_list = ['%d' % x for x in range(0, 10)]
    _sleepmode_list = ['Off'] + ['1:%d' % x for x in range(1, 11)]
    _sidekey_list = ['None', 'Monitor', 'Scan',
                     'Alarm', 'Bluetooth', 'Weather']
    _channel_list = ['%d' % x for x in range(1, _upper + 1)]

    _freqband_list = []
    for x in VALID_BANDS:
        _freqband_list.append(str(int(x[0] / 1000000)) + '-' +
                              str(int(x[1] / 1000000)))

    def sync_in(self):
        """Download from radio"""
        try:
            data = do_download(self)
        except errors.RadioError:
            # Pass through any real errors we raise
            raise
        except Exception:
            # If anything unexpected happens, make sure we raise
            # a RadioError and log the problem
            LOG.exception('Unexpected error during download')
            raise errors.RadioError('Unexpected error communicating '
                                    'with the radio')
        finally:
            _exit_programming_mode(self)

        self._mmap = data
        self.process_mmap()

    def sync_out(self):
        """Upload to radio"""
        try:
            do_upload(self)
        except Exception:
            # If anything unexpected happens, make sure we raise
            # a RadioError and log the problem
            LOG.exception('Unexpected error during upload')
            raise errors.RadioError('Unexpected error communicating '
                                    'with the radio')
        finally:
            _exit_programming_mode(self)

    def _decode_tone(self, val):
        if val == 16665 or val == 0:
            return '', None, None
        elif val >= 12000:
            return 'DTCS', val - 12000, 'R'
        elif val >= 8000:
            return 'DTCS', val - 8000, 'N'
        else:
            return 'Tone', val / 10.0, None

    def _encode_tone(self, memval, mode, value, pol):
        if mode == "":
            memval[0].set_raw(0xFF)
            memval[1].set_raw(0xFF)
        elif mode == 'Tone':
            memval.set_value(int(value * 10))

        elif mode == 'DTCS':
            flag = 0x80 if pol == 'N' else 0xC0
            memval.set_value(value)
            memval[1].set_bits(flag)
        else:
            raise Exception("Internal error: invalid mode `%s'" % mode)

    def get_settings(self):
        _settings = self._memobj.settings
        basic = RadioSettingGroup("basic", 'Settings')
        group = RadioSettings(basic)

        # format raw firmware for display
        def _fw(version):
            ver = 'v'
            for i in version:
                ver += '%d' % i + '.'
            return ver[:-3]

        # Firmware version
        rs = RadioSettingValueString(6, 6, _fw(_settings.firmware))
        rs.set_mutable(False)
        rset = RadioSetting('setting.firmware', 'Firmware Version', rs)
        rset.set_doc('Radio Firmware Version')
        basic.append(rset)

        # Voice prompts
        rs = RadioSettingValueList(self._voice_list,
                                   current_index=_settings.voice)
        rset = MemSetting('settings.voice', 'Voice Prompts', rs)
        rset.set_doc('Radio Voice Prompts language')
        basic.append(rset)

        # TOT
        rs = RadioSettingValueList(self._tot_list,
                                   current_index=_settings.tot)
        rset = MemSetting('settings.tot', 'Time Out Timer', rs)
        rset.set_doc('Radio TX Time Out Timer value')
        basic.append(rset)

        # VOX level
        rs = RadioSettingValueList(self._voxlevel_list,
                                   current_index=_settings.vox)
        rset = MemSetting('settings.vox', 'VOX Level', rs)
        rset.set_doc('Radio VOX Level senesitivity value')
        basic.append(rset)

        # Squelch level
        rs = RadioSettingValueList(self._squelchlevel_list,
                                   current_index=_settings.squelch)
        rset = MemSetting('settings.squelch', 'Squelch Level', rs)
        rset.set_doc('Radio Squelch Level value')
        basic.append(rset)

        # Sleep mode
        rs = RadioSettingValueList(self._sleepmode_list,
                                   current_index=_settings.sleep)
        rset = MemSetting('settings.sleep', 'Sleep Mode', rs)
        rset.set_doc('Radio Sleep Mode power saving ratio value')
        basic.append(rset)

        # Sidekey 1
        rs = RadioSettingValueList(self._sidekey_list,
                                   current_index=_settings.sidekey1)
        rset = MemSetting('settings.sidekey1', 'Sidekey 1 Long Pess', rs)
        rset.set_doc('Radio Sidekey 1 Long Press assigned action value')
        basic.append(rset)

        # Sidekey 2
        rs = RadioSettingValueList(self._sidekey_list,
                                   current_index=_settings.sidekey2)
        rset = MemSetting('settings.sidekey2', 'Sidekey 2 Long Press', rs)
        rset.set_doc('Radio Sidekey 2 Long Press assigned action value')
        basic.append(rset)

        # Selected default channel (setting not in factory CPS)
        rs = RadioSettingValueList(self._channel_list,
                                   current_index=_settings.channel)
        rset = MemSetting('settings.channel', 'Default Channel', rs)
        rset.set_doc('Radio Channel that is selected by Default at power-on')
        basic.append(rset)

        return group

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.experimental = \
            ('This driver is a BETA version ONLY for the TIDRADIO '
             'TD-M11 22 running Firmware v0.9.4\n'
             '\n'
             'Please save an unedited copy of your first successful\n'
             'download to a CHIRP Radio Images(*.img) file.\n\n'
             'PROCEED AT YOUR OWN RISK!'
             )
        rp.pre_download = (dedent("""\
            1. Turn radio off.
            2. Connect cable to mic/spkr connector.
            3. Make sure connector is firmly connected.
            4. Turn radio on (volume may need to be set at 100%).
            5. Ensure that the radio is tuned to channel with no activity.
            6. Click OK to download image from device."""))
        rp.pre_upload = (dedent("""\
            1. Turn radio off.
            2. Connect cable to mic/spkr connector.
            3. Make sure connector is firmly connected.
            4. Turn radio on (volume may need to be set at 100%).
            5. Ensure that the radio is tuned to channel with no activity.
            6. Click OK to upload image to device."""))
        return rp

    def get_features(self):
        rf = get_default_features(self)
        rf.valid_bands = self.VALID_BANDS
        rf.valid_modes = ["FM", "NFM"]  # 25kHz, 12.5kHz
        rf.valid_tuning_steps = self._steps
        rf.has_name = False
        return rf

    def get_memory(self, number):
        """Get the mem representation from the radio image"""
        _mem = self._memobj.memory[number - 1]

        # Create a high-level memory object to return to the UI
        mem = chirp_common.Memory()

        # Memory number
        mem.number = number

        if _mem.get_raw()[:1] == b"\xFF":
            mem.empty = True
            return mem

        # Freq and offset
        mem.freq = int(_mem.rxfreq) * 10
        if mem.freq == 0:
            mem.empty = True
        # tx freq can be blank
        if _mem.txfreq.get_raw() == b"\xFF\xFF\xFF\xFF":
            # TX freq not set
            mem.offset = 0
            mem.duplex = "off"
        else:
            # TX freq set
            offset = (int(_mem.txfreq) * 10) - mem.freq
            if offset != 0:
                if chirp_common.is_split(self.get_features().valid_bands,
                                         mem.freq, int(_mem.txfreq) * 10):
                    mem.duplex = "split"
                    mem.offset = int(_mem.txfreq) * 10
                elif offset < 0:
                    mem.offset = abs(offset)
                    mem.duplex = "-"
                elif offset > 0:
                    mem.offset = offset
                    mem.duplex = "+"
            else:
                mem.offset = 0

        txtone = rxtone = None
        txtone = self._decode_tone(int(_mem.txtone))
        rxtone = self._decode_tone(int(_mem.rxtone))
        chirp_common.split_tone_decode(mem, txtone, rxtone)

        if not _mem.scan:
            mem.skip = 'S'

        try:
            mem.power = self._power_levels[_mem.txpower]
        except Exception:
            LOG.error('Channel %d: get_memory: unhandled power level: 0x%02x' %
                      (mem.number, _mem.txpower))

        mem.mode = _mem.narrow and "NFM" or "FM"

        mem.extra = RadioSettingGroup("Extra", "extra")

        # BCL (Busy Channel Lockout)
        rs = RadioSettingValueList(self._bcl_list,
                                   current_index=_mem.bcl)
        rset = RadioSetting("bcl", "BCL", rs)
        mem.extra.append(rset)

        # Jump Freq
        rs = RadioSettingValueList(
            self._jumpfreq_list,
            current_index=_mem.jumpfreq)
        rset = RadioSetting("jumpfreq", "Jump Freq", rs)
        mem.extra.append(rset)

        # Compand
        rs = RadioSettingValueList(
            self._compand_list,
            current_index=_mem.compand)
        rset = RadioSetting("compand", "Compand", rs)
        mem.extra.append(rset)

        # Scramble
        rs = RadioSettingValueList(self._scramble_list,
                                   current_index=_mem.scramble)
        rset = RadioSetting("scramble", "Scramble", rs)
        mem.extra.append(rset)

        return mem

    def set_memory(self, mem):
        _mem = self._memobj.memory[mem.number - 1]

        if mem.empty:
            _mem.set_raw("\xff" * 16)
            return

        _mem.set_raw("\x00" * 16)

        _mem.rxfreq = mem.freq / 10

        if mem.duplex == "off":
            _mem.txfreq.fill_raw(b"\xFF")
        elif mem.duplex == "split":
            _mem.txfreq = mem.offset / 10
        elif mem.duplex == "+":
            _mem.txfreq = (mem.freq + mem.offset) / 10
        elif mem.duplex == "-":
            _mem.txfreq = (mem.freq - mem.offset) / 10
        else:
            _mem.txfreq = mem.freq / 10

        txtone, rxtone = chirp_common.split_tone_encode(mem)
        self._encode_tone(_mem.txtone, *txtone)
        self._encode_tone(_mem.rxtone, *rxtone)

        _mem.scan = mem.skip != "S"
        _mem.narrow = mem.mode == "NFM"

        try:
            _mem.txpower = self._power_levels.index(mem.power)
        except Exception:
            LOG.error('Channel %d: set_memory: unhandled power level: %s' %
                      (mem.number, mem.power))

        try:
            _mem.narrow = self.get_features().valid_modes.index(mem.mode)
        except Exception:
            LOG.error('Channel %d: set_memory: unhandled mode: %s' %
                      (mem.number, mem.mode))

        for setting in mem.extra:
            setattr(_mem, setting.get_name(), setting.value)

    def validate_memory(self, mem):
        msgs = []
        in_range = chirp_common.in_range

        if not in_range(mem.freq, self.VALID_BANDS):
            s = ''
            for x in self.VALID_BANDS:
                s += str(int(x[0] / 1000000)) + '-' + \
                    str(int(x[1] / 1000000)) + ', '
            s = '\n' + s[:-2]
            msgs.append(chirp_common.ValidationError(s))

        return super().validate_memory(mem) + msgs

    def set_settings(self, settings):
        # apply all Memsettings
        all_other_settings = settings.apply_to(self._memobj)
        for setting in all_other_settings:
            if setting.has_apply_callback():
                # use callbacks on Radiosettings that need postprocessing
                setting.run_apply_callback()

    def process_mmap(self):
        mem_format = MEM_FORMAT % self._mem_params
        self._memobj = bitwise.parse(mem_format, self._mmap)


# @directory.register
# class TDM11_16(TDM11_22):
#     # ==========
#     # Notice to developers:
#     # The TD-M11 16 for EU PMR446  support in this driver is currently based
#     # on Firmware v0.9.4
#     # ==========
#     """TIDRADIO TD-M11 16"""
#     VENDOR = "TIDRADIO"
#     MODEL = "TD-M11"
#     VARIANT = '16 PMR'  # EU PMR

#     _upper = 16
#     _mem_params = (_upper)

#     _ranges = [
#         (0x0000, 0x0100),  # 16ea 16 byte channels
#         (0x0300, 0X033F)  # settings and password
#     ]

#     _channel_list = ['%d' % x for x in range(1, _upper + 1)]

#     @classmethod
#     def get_prompts(cls):
#         rp = chirp_common.RadioPrompts()
#         rp.experimental = \
#             ('This driver is a BETA version ONLY for the TIDRADIO '
#              'TD-M11 16 running Firmware v0.x.y\n'
#              '\n'
#              'Please save an unedited copy of your first successful\n'
#              'download to a CHIRP Radio Images(*.img) file.\n\n'
#              'PROCEED AT YOUR OWN RISK!'
#              )
#         return rp
