from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

R1 = 10
R2 = 22
SAMPLE_RATE = 10000

def main():
    cwd = Path("./testing")
    pressure_file = Path("./testing")
    for f in cwd.iterdir():
        if f.name.startswith("Flight"):
            pressure_file = f
            break
    if pressure_file.is_dir():
        return
    print(pressure_file)

    dt = np.dtype(np.uint16, metadata={'byteorder':'<'})
    array = np.fromfile(pressure_file, dtype=dt)
    timestamps = array[0::2]
    pressure_voltage = array[1::2]
    pressure_voltage = pressure_voltage * 3.3 * (R1 + R2) / (4095 * R2)
    pressure = 0.0 + (pressure_voltage - 0.5) * (1600-0) / (4.5 - 0.5) + 6+14
    pressure = pressure[42*SAMPLE_RATE:47*SAMPLE_RATE]
    t = np.arange(0, pressure.shape[0]) / SAMPLE_RATE

    fig, ax = plt.subplots()
    # ax.plot(timestamps)
    ax.plot(t, pressure)

    # ax.set_ybound(0, 4096)
    # ax.yaxis.set_major_locator(ticker.MultipleLocator(1024, offset=0))
    # ax.yaxis.set_minor_locator(ticker.MultipleLocator(256))

    # ax.set_ybound(0, 5)
    # ax.yaxis.set_major_locator(ticker.MultipleLocator(1.25, offset=0))
    # ax.yaxis.set_minor_locator(ticker.MultipleLocator(0.25))

    ax.set_title("Ground Test 2")
    ax.set_ybound(0, 1600)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pressure (psi)")
    ax.yaxis.set_major_locator(ticker.MultipleLocator(400, offset=0))
    ax.yaxis.set_minor_locator(ticker.MultipleLocator(100))
    ax.grid(True, which='both')

    plt.show() 

if __name__ == "__main__":
    main()