from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# Converts packed 12bit data and plots it.

IS_TESTFILE = True  # Whether data should contain the test pattern

def main():
    cwd = Path("./testing")
    pressure_file = Path("./testing")
    for f in cwd.iterdir():
        if f.name.startswith("Pressure"):
            pressure_file = f
            break
    if pressure_file.is_dir():
        return

    array8 = np.fromfile(pressure_file, dtype=np.uint8)
    array12 = np.empty(array8.shape[0]//3*2, dtype=np.uint16)
    for i in range(array8.shape[0]//3):
        fst_uint8=np.uint16(array8[i*3])
        mid_uint8=np.uint16(array8[i*3+1])
        lst_uint8=np.uint16(array8[i*3+2])
        
        array12[i*2] =   (fst_uint8 << 4) + (mid_uint8 >> 4)
        array12[i*2+1] = ((mid_uint8 % 16) << 8) + lst_uint8
    
    del array8
    if IS_TESTFILE:
        j = 0
        for i in range(array12.shape[0]):
            if j % 4096 != array12[j]:
                print(f"Incorrect value at x={i}")
                j = array12[j]
            j += 1
    fig, ax = plt.subplots()
    ax.plot(array12)
    ax.set_ybound(0, 4096)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1024, offset=0))
    ax.yaxis.set_minor_locator(ticker.MultipleLocator(256))
    plt.show() 

if __name__ == "__main__":
    main()