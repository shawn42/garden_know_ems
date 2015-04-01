
task :compile do
  sh 'make default'
  sh 'ruby tools/init_packet.rb _build/nrf51822_xxac_s110.bin _build/nrf51822_xxac_s110.dat'
  cp '_build/nrf51822_xxac_s110.bin', '/Users/shawn42/Dropbox/beacon/'
  cp '_build/nrf51822_xxac_s110.dat', '/Users/shawn42/Dropbox/beacon/'
end

task default: :compile
