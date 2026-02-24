from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

def main():
    cwd = Path("./testing")
    pressure_file = Path("./testing")
    for f in cwd.iterdir():
        if f.name.startswith("Flight"):
            pressure_file = f
            break
    if pressure_file.is_dir():
        return

    dt = np.dtype(np.uint16, metadata={'byteorder':'<'})
    array = np.fromfile(pressure_file, dtype=dt)
    timestamps = array[0::2]
    pressure = array[1::2]


    fig, ax = plt.subplots()
    # ax.plot(timestamps)
    ax.plot(pressure)
    ax.set_ybound(0, 4096)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1024, offset=0))
    ax.yaxis.set_minor_locator(ticker.MultipleLocator(256))
    plt.show() 

if __name__ == "__main__":
    main()