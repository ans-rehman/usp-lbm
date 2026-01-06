# Input string
s = input("Enter hex string: ")  # e.g. 6f8755f9b9

# Make sure the string length is even
if len(s) % 2 != 0:
    print("Error: input length must be even.")
    exit(1)

# Convert and print
hex_bytes = ["0x" + s[i:i+2] for i in range(0, len(s), 2)]
print(" ".join(hex_bytes))
