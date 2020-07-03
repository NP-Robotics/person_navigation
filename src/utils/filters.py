def highpass_filter(value, threshold):
    if value<threshold and value > -threshold:
        return 0
    else:
        return value


if __name__ == "__main__":
    print(lowpass_filter(3,4))
    print(lowpass_filter(3,-4))
    print(lowpass_filter(3, 2))
    print(lowpass_filter(-3, -2))