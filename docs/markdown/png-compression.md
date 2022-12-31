# PNG Compression

```
# ratio can be from 0 (no compression) to 9 (highest)
ratio = 0
cv2.imwrite(filename, data, [cv2.IMWRITE_PNG_COMPRESSION, ratio])
```

Supported formats of interest:

- JPEG files - *.jpeg, *.jpg, *.jpe (see the Note section)
- JPEG 2000 files - *.jp2 (see the Note section)
- Portable Network Graphics - *.png (see the Note section)
- WebP - *.webp (see the Note section)
- Portable image format - *.pbm, *.pgm, *.ppm *.pxm, *.pnm (always supported)


## References

- OpenCV [docs](https://docs.opencv.org/master/d4/da8/group__imgcodecs.html#gga292d81be8d76901bff7988d18d2b42acad2548321c69ab9c0582fd51e75ace1d0)
