import sounddevice as sd
import soundfile as sf
from networktables import NetworkTablesInstance
import time


if __name__ == "__main__":
    # Start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    ntinst.startClientTeam(6328)
    ntinst.startDSClient()

    # Check for enabled
    is_playing = False
    while True:
        time.sleep(0.1)
        controlData = ntinst.getEntry("/FMSInfo/FMSControlData").getNumber(0)
        is_playing_new = controlData % 2 == 1
        if not is_playing and is_playing_new:
            data, fs = sf.read("crab_rave.wav", dtype='float32')
            sd.play(data, fs, loop=True)
        elif is_playing and not is_playing_new:
            sd.stop()
        is_playing = is_playing_new
