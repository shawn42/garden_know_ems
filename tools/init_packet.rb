require 'digest/crc16_ccitt'
File.open ARGV[1], 'wb' do |f|
  hardware_version = [0xFF, 0xFF]
  hardware_revision = [0xFF, 0xFF]
  application_version = [0xFF, 0xFF, 0xFF, 0xFF]
  softdevice_len = [0x01, 0x00]
  softdevice_array = [0xFE, 0xFF]

  f.write hardware_version.pack("C*")
  f.write hardware_revision.pack("C*")
  f.write application_version.pack("C*")
  f.write softdevice_len.pack("C*")
  f.write softdevice_array.pack("C*")

  crc = [Digest::CRC16CCITT.checksum(File.read(ARGV[0]))].pack("S*")
  f.write crc
end
