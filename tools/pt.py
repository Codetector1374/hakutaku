def breakdown_address(addr):
    addr >>= 12
    pt = addr & 0x1FF
    addr >>= 9
    pd = addr & 0x1FF
    addr >>= 9
    pdp = addr & 0x1FF
    addr >>= 9
    pml4 = addr & 0x1FF
    print('PML4 = %d, PDP = %d, PD = %d, pt = %d)' % (pml4, pdp, pd, pt))

def convert( aString ):
    if aString.startswith("0x") or aString.startswith("0X"):
        return int(aString,16)
    elif aString.startswith("0"):
        return int(aString,8)
    else:
        return int(aString)

if __name__ == '__main__':
    while True:
        breakdown_address(convert(input()))
