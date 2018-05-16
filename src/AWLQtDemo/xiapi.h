#define DWORD double
#define HANDLE int
typedef struct
{
        DWORD         size;      // Size of current structure on application side. When xiGetImage is called and size>=SIZE_XI_IMG_V2 then GPI_level, tsSec and tsUSec are filled.
        DWORD         bp_size;   // Filled buffer size. When buffer policy is set to XI_BP_SAFE, xiGetImage will fill this field with current size of image data received.
        DWORD         width;     // width of incoming image.
        DWORD         height;    // height of incoming image.
        DWORD         nframe;    // frame number(reset by exposure, gain, downsampling change).
        DWORD         tsSec;     // TimeStamp in seconds
        DWORD         tsUSec;    // TimeStamp in microseconds
        DWORD         GPI_level; // Input level

}XI_IMG;

