import argparse
import subprocess
import time

CONNECTION_ATTEMPTS = 0
MAX_NUMBER_OF_ATTEMPTS = 15


def connect_device(ssid, password=None, verbose=True):
    """
    Function which attempts to establish connection with
    a WiFi device with the same ssid and password if provided.
    In case it fails, 4 more attempts will be made.
    @params:
        - ssid: ssid of the WiFi device
        - password: of the WiFi device
        - verbose: when set to true it print useful information. Default = True
    Returns True if the connection was establish, False otherwise.
    """

    print(
        f"[INFO] - Establishing WiFi connection to {ssid} ...") if verbose else None

    if password:
        response = subprocess.run(
            f"nmcli dev wifi connect '{ssid}' password {password}", shell=True
        )
    else:
        response = subprocess.run(f"nmcli dev wifi connect {ssid}", shell=True)

    if response.returncode != 0:
        print(
            f"[ERROR] - Unable to establish connection with {ssid}"
        ) if verbose else None

        global CONNECTION_ATTEMPTS
        CONNECTION_ATTEMPTS += 1
        if CONNECTION_ATTEMPTS > MAX_NUMBER_OF_ATTEMPTS:
            return False

        time.sleep(1)
        print("[INFO] - Retrying...")
        return connect_device(ssid, password, verbose)

    CONNECTION_ATTEMPTS = 0
    print("[INFO] - Connected!") if verbose else None
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-id", "--ssid", required=True, help="Enter the WiFi SSID")
    ap.add_argument("-pw", "--password", default=None,
                    help="Enter the WiFi password")

    args = vars(ap.parse_args())

    connect_device(args["ssid"], args["password"])


if __name__ == "__main__":
    main()
