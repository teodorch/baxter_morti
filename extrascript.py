MIN_MATCH_COUNT = 20

def get_warped(image, pts, rect):

    p1, p2, p3, p4 = pts
    tl = p2[0]
    tr = p1[0]
    bl = p3[0]
    br = p4[0]

    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
     
    # ...and now for the height of our new image
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
     
    # take the maximum of the width and height values to reach
    # our final dimensions
    maxWidth = max(int(widthA), int(widthB))
    maxHeight = max(int(heightA), int(heightB))
    
    rect[0] = tl
    rect[1] = tr
    rect[2] = br
    rect[3] = bl
    #rect = order_points(rect)
    #x, y, w, h = cv2.boundingRect(dst)
    #y = 0
    dst2 = np.array([[0, 0],[maxWidth - 1, 0],	[maxWidth - 1, maxHeight - 1],	[0, maxHeight - 1]], dtype = "float32")
    
    M = cv2.getPerspectiveTransform(rect, dst2)
    warp = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warp

def drawMatches(img1, kp1, img2, kp2, matches, name):
    """
    My own implementation of cv2.drawMatches as OpenCV 2.4.9
    does not have this function available but it's supported in
    OpenCV 3.0.0

    This function takes in two images with their associated 
    keypoints, as well as a list of DMatch data structure (matches) 
    that contains which keypoints matched in which images.

    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.

    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.

    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1, img1, img1])

    # Place the next image to the right of it
    out[:rows2,cols1:] = np.dstack([img2, img2, img2])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them

    for mat in matches:
        
        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        (x1,y1) = kp1[img1_idx].pt
        (x2,y2) = kp2[img2_idx].pt

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (0, 0, 255), 2)   
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (0, 0, 255), 2)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 2)

    # Show the image
    cv2.imshow('Matched Features', out)
    #cv2.imwrite("homography_" +name, out)
    cv2.waitKey(0)
    cv2.destroyWindow('Matched Features')

    # Also return the image if you'd like a copy
    return out
    
def match(img2_color):

    #img1 = cv2.imread('right.jpg',0)          # queryImage
    #img2 = cv2.imread('./data/box2.jpg',0) # trainImage
    img2 = cv2.cvtColor(img2_color, cv2.COLOR_BGR2GRAY)
    #img2_color = cv2.imread('./data/box2.jpg')
    # Initiate SURF detector
    surf = cv2.SURF()
    path = "images/train_data/"
    rect = np.zeros((4, 2), dtype = "float32")
    
    color1 = (255, 0, 0)
    color2 = (255, 0, 255)
    kp2, des2 = surf.detectAndCompute(img2,None)
    for obj in os.listdir(path):
        if (obj == "Thumbs.db"):
            continue
        path2 = path + obj + "/"
        for name in reversed(os.listdir(path2)):
            if (name == "Thumbs.db"):
                continue

            img1 = cv2.imread(path2+name,0) 
            #img1 = cv2.imread("distance.jpg",0) 
            # find the keypoints and descriptors with SIFT
            kp1, des1 = surf.detectAndCompute(img1,None)


            FLANN_INDEX_KDTREE = 0
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)

            flann = cv2.FlannBasedMatcher(index_params, search_params)

            matches = flann.knnMatch(des1,des2,k=2)

            # store all the good matches as per Lowe's ratio test.
            good = []
            for m,n in matches:
                if m.distance < 0.8*n.distance:
                    good.append(m)

            thresh = len(kp1) * 20 / 100

            #drawMatches(img1, kp1, img2, kp2, good, name)
            #img3 = cv2.drawKeypoints(img1,kp1,None,(255,0,0),4)
            #cv2.imshow("keypoints.jpg", img3)        print len(kp1)

            if len(good)>MIN_MATCH_COUNT:

                src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ])
                dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ])

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matchesMask = mask.ravel().tolist()

                h,w = img1.shape
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M)
                
                if obj == "object_1":
                    color = color1
                else:
                    color = color2
                
                cv2.polylines(img2_color,[np.int32(dst)],True,color,3)    
                warp = get_warped(img2_color, dst, rect)
                #cv2.imshow("closest_warped.jpg", warp)
                #cv2.imwrite("angle" +name, img2_color)
                #cv2.imwrite("closer_warped.jpg", warp)
                #cv2.imwrite("13_keypoints_warped.jpg", warp)
                #print "Matched key points ", len(good)
                #if (len(good) < thresh and len(os.listdir(path)) < 50):
                #    cv2.imwrite(path + "train_%03d.jpg" % i[0], warp)
                #    i[0] += 1 
                #    print i
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                break;
    cv2.imshow("closest_detect.jpg", img2_color)
    cv2.imwrite("result2.jpg", img2_color)
    print "asd"
    cv2.waitKey(0)
    cv2.destroyAllWindows()
