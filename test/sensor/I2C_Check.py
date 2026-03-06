import board
import busio

def main():
    print("I2C.")
    i2c = board.I2C()
    print("I2C OK:", i2c)

if __name__ == "__main__":
    main()
