import os
import os.path as osp
import sys
import spectral
from PIL import Image
import numpy as np

def ceildiv(a, b):
    return -(a // -b)

def convert_hdr_to_pngs(hdrfilepath, 

                        # https://pillow.readthedocs.io/en/stable/handbook/concepts.html#concept-modes
                        #  "1", "L;1", "L;2", "L;4", "L", "LA", "I", "I;16",
                        #  "P;1", "P;2", "P;4", "P", "RGB", "RGBA"
                        # You will have to add pngprops below
                        pngmode = 'RGB', # One of the pillow supported modes

                        pngmode_props = {
                            'RGB': dict(
                                N = 3, # max channels per png file
                                dtype = 'u1', # dtype for saving png files
                            ),
                            'I;16': dict(
                                N = 1, # max channels per png file
                                dtype = 'i2', # dtype for saving png files
                            )
                        }
                       ):
    """
    Converts HDR images into 3-channel pngs for easy visualization
    """
    pngprops = pngmode_props[pngmode]
    N, dtype = [pngprops[k] for k in 'N dtype'.split()]
    dtmax = np.iinfo(dtype).max

    hdrbinfile, ext = osp.splitext(hdrfilepath)
    assert os.path.exists(hdrbinfile), f'{hdrbinfile} must exist'
    img = spectral.open_image(hdrfilepath)
    imgfiledir = f'{hdrbinfile}_files'
    os.makedirs(imgfiledir, exist_ok=True)
    for i in range(ceildiv(img.shape[2], N)):
        channel_start = i*N
        channel_end = min((i+1)*N, img.shape[2])
        imgslice = img[:, :, channel_start:channel_end]
        imgslice_min = imgslice.min()
        # Change range to 0, 255. This WILL cause data loss
        imgscaled = ((imgslice - imgslice_min) / (imgslice.max() - imgslice_min) * dtmax)
        imgint = imgscaled.astype(dtype)
        if imgint.shape[2] != N:
            # if not 3 channels then 
            nc = imgint.shape[2]
            imgint = np.concatenate((imgint,
                                     np.zeros((*imgint.shape[:2], N-nc),
                                             dtype=dtype)),
                                   axis=2)
        if N == 1:
            imgint = imgint.squeeze()
        Image.fromarray(imgint, mode=pngmode).save(
            osp.join(imgfiledir, f'{pngmode}-{channel_start:03d}-{channel_end:03d}.png'))

def listget(lst, i, default):
    return lst[i] if i < len(lst) else default

if __name__ == '__main__':
    hdrfilepath = listget(sys.argv, 1, 'data/raw_14864_rd_rf_or.hdr')
    convert_hdr_to_pngs(hdrfilepath)

