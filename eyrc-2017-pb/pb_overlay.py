import cv2
import numpy as np
import os
def blend_transparent(face_img, overlay_t_img):
    overlay_img = overlay_t_img[:,:,:3]
    overlay_mask = overlay_t_img[:,:,3:] 
    background_mask = 255 - overlay_mask
    overlay_mask = cv2.cvtColor(overlay_mask, cv2.COLOR_GRAY2BGR)
    background_mask = cv2.cvtColor(background_mask, cv2.COLOR_GRAY2BGR)
    face_part = (face_img * (1 / 255.0)) * (background_mask * (1 / 255.0))
    overlay_part = (overlay_img * (1 / 255.0)) * (overlay_mask * (1 / 255.0))   
    return np.uint8(cv2.addWeighted(face_part, 255.0, overlay_part, 255.0, 0.0))
def main(path):
    print path
    img=cv2.imread("Plantation\\Plantation.png")
    flower=cv2.imread(path,-1)
    ### ROI of Zones ###
    """
    pz_a:
    if count<=2:
        x=int(abs((475-593))/2)
        y=int(abs((230-298)))
        yi=abs(210)
        xi=abs(400)
    else:
        x=int(abs((370-593))/count)
        y=int(abs((230-298)))
        yi=abs(210)
        xi=abs(320)
    dist_fact=7
    pz_b:
    if count<=2:
        x=int(int(abs((120-189))/2))
        y=int(int(abs((221-257))))
        yi=abs(221)
        xi=abs(95)
    else:
        x=int(int(abs((70-189))/count))
        y=int(int(abs((221-257))))
        yi=abs(221)
        xi=abs(65)
    dist_fact=3
    pz_c:
    if count<=2:
        x=int(int(abs((300-380))/2))
        y=int(int(abs((165-200))))
        yi=abs(160)
        xi=abs(275)
    else:
        x=int(int(abs((252-380))/count))
        y=int(int(abs((165-200))))
        yi=abs(160)
        xi=abs(252)
    dist_fact=3
    pz_d:
    if count<=2:
        x=int(int(abs((560-625))/2))
        y=int(int(abs((190-220))))
        yi=abs(185)
        xi=abs(540)
    else:
        x=int(int(abs((535-625))/count))
        y=int(int(abs((195-215))))
        yi=abs(190)
        xi=abs(535)
    dist_fact=3
    """
    count=4
    if count<=2:
        x=int(int(abs((540-625))/2))
        y=int(int(abs((180-220))))
        yi=abs(170)
        xi=abs(520)
    else:
        x=int(int(abs((510-625))/count))
        y=int(int(abs((180-220))))
        yi=abs(170)
        xi=abs(505)
    dist_fact=3
    rflower=cv2.resize(flower,(x,y))
    for c in range(1,count+1):
        img[yi:yi+y,(xi+c*dist_fact)+(c-1)*x:(xi+c*dist_fact)+c*x,:] = blend_transparent(img[yi:yi+y,(xi+c*dist_fact)+(c-1)*x:(xi+c*dist_fact)+c*x,:],rflower)
        
    cv2.imshow('roi',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == "__main__":
    mypath = ".\\Seedlings\\"
    onlyfiles = [os.path.join(mypath, f) for f in os.listdir(mypath) if f.endswith(".png")]
    for fp in onlyfiles:
        main(fp)
