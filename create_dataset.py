import serial
import numpy as np
import cv2
import sys

def read_image_from_serial(ser, image_size=(32, 32)):
    """
    Reads a single image from the serial connection.
    """
    image = np.zeros((image_size[0], image_size[1], 3), dtype=np.uint8)
    bin_image = np.zeros((image_size[0], image_size[1]), dtype=np.uint8)
    pixel_count = 0

    while True:
        # Read a line from Serial
        line = ser.readline().decode('utf-8').strip()
        #print(line)
        
        if line == "END":  # End marker for the current image
            print(pixel_count)
            break
        
        # Parse the R, G, B values
        try:
            #print(line)
            r, g, b = map(int, line.split(","))
            y = pixel_count // image_size[1]
            x = pixel_count % image_size[1]
            image[y, x] = [b, g, r]  # OpenCV uses BGR format
            bin_image[y, x] = 255 if ((r != -128) | (g != -128) | (b != -128)) else 0
            pixel_count += 1
        except ValueError:
            pass
            #print(f"Invalid line: {line}")

    if pixel_count != image_size[0] * image_size[1]:
        print("Warning: Incomplete image received.")
    
    return image, bin_image

def display_images_continuously(port, baudrate, image_size=(32, 32)):
    """
    Continuously receives and displays images from the serial connection.
    """
    ser = serial.Serial(port, baudrate, timeout=1)
    print("Waiting for image data...")

    img_data = []
    bin_img_data = []
    label_data = []

    stop = False

    try:
        while True:
            print("Receiving a new image...")
            image, bin_image = read_image_from_serial(ser, image_size)
            image_large = cv2.resize(bin_image, (512, 512), interpolation = cv2.INTER_NEAREST_EXACT)
            cv2.imshow("Received Image", image_large)

            key = cv2.waitKey(33)
            while key == -1:
                key = cv2.waitKey(33)
                if key == ord('z'):
                    break
                elif key == ord('w'):
                    img_data.append(image)
                    bin_img_data.append(bin_image)
                    label_data.append(0)
                    print("Up collected: %d" % len(label_data))
                elif key == ord('a'):
                    img_data.append(image)
                    bin_img_data.append(bin_image)
                    label_data.append(1)
                    print("Left collected: %d" % len(label_data))
                elif key == ord('s'):
                    img_data.append(image)
                    bin_img_data.append(bin_image)
                    label_data.append(2)
                    print("Down collected: %d" % len(label_data))
                elif key == ord('d'):
                    img_data.append(image)
                    bin_img_data.append(bin_image)
                    label_data.append(3)
                    print("Right collected: %d" % len(label_data))
                elif key == ord('q'):
                    stop = True
                    print("Stop initiated")
                    break

            if stop:
                print("Yes stop")
                break
    finally:

        img_data = np.array(img_data)
        bin_img_data = np.array(bin_img_data)
        label_data = np.array(label_data)
        old_img_data = np.load("training_data_v2.npy")
        old_img_bin_data = np.load("training_data_bin_v2.npy")
        old_label_data = np.load("training_labels_v2.npy")
        img_data = np.concatenate((old_img_data, img_data))
        bin_img_data = np.concatenate((old_img_bin_data, bin_img_data))
        label_data = np.concatenate((old_label_data, label_data))
        np.save("training_data_v2.npy", img_data)
        np.save("training_data_bin_v2.npy", bin_img_data)
        np.save("training_labels_v2.npy", label_data)

        ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Specify your serial port and baudrate
    serial_port = "COM9"  # Replace with your Arduino's port
    baud_rate = 9600

    # Start continuous reception
    display_images_continuously(serial_port, baud_rate)

    train_data = np.load("training_data_v2.npy")
    train_bin_data = np.load("training_data_bin_v2.npy")
    train_labels = np.load("training_labels_v2.npy")

    print(train_data.shape)
    print(train_bin_data.shape)
    print(train_labels)

