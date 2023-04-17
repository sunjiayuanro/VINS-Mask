import grpc
import common_pb2
import common_pb2_grpc
import service_pb2
import service_pb2_grpc
import time
from concurrent import futures
from pathlib import Path
import argparse
import cv2
import matplotlib.cm as cm
import torch
import numpy as np
import time
import base64

import torch

from superpoint import SuperPoint

from utils import (AverageTimer, VideoStreamer,
                          make_matching_plot_fast, frame2tensor)

torch.set_grad_enabled(True)

import grpc.experimental
from grpc.experimental import unary_unary

# Use the experimental methods
grpc.experimental.unary_unary

config = {}

device = 'cuda' if torch.cuda.is_available() else 'cpu'
print('Running inference on device \"{}\"'.format(device))

class FeatService(service_pb2_grpc.RpcServicer):

    def __init__(self):
        self.superpoint = SuperPoint(config.get('superpoint', {})).to(device=device)

    # Check Communication
    def CheckCommunication(self, request, context):
        print("Im fine.")
        return common_pb2.CommandFeedback(status=common_pb2.StateEnum.ACTIVE)
    
    # SuperPoint
    def GetSuperPoint(self, request, context):
        # start = time.time()
        im_str = base64.b64decode(request.data)
        nparr = np.fromstring(im_str, np.uint8)
        img_decode = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
        im_tensor = frame2tensor(img_decode, device)
        pred0 = self.superpoint({"image": im_tensor})
        pred = {}
        pred = {**pred, **{k+'0': v for k, v in pred0.items()}}
    
        kpts = pred['keypoints0'][0].cpu().numpy()
        points = []
        for pt in kpts:
            points.append(common_pb2.Point2D(x=pt[0],y=pt[1]))
        stamp = time.time()
        # print("GetSuperPoint: ",stamp-start)
        return common_pb2.PointCloudXY(points=points,stamp=stamp)

def serve():
    # rpc service
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2)) 
    service_pb2_grpc.add_RpcServicer_to_server(FeatService(), server)
    server.add_insecure_port('[::]:50055')
    server.start()
    server.wait_for_termination()

from PIL import Image
from matplotlib import pyplot as pl; pl.ion()
from scipy.ndimage import uniform_filter
smooth = lambda arr: uniform_filter(arr, 3)
import pdb

def transparent(img, alpha, cmap, **kw):
    from matplotlib.colors import Normalize
    colored_img = cmap(Normalize(clip=True,**kw)(img))
    colored_img[:,:,-1] = alpha
    return colored_img


if __name__ == '__main__':
    serve()
    
    # fig = pl.figure("viz")
    # kw = dict(cmap=pl.cm.RdYlGn, vmax=1)
    # # crop = (slice(10,-10 or 1),)*2

    # R = 9
    # superpoint = SuperPoint(config.get('superpoint', {})).to(device=device)
    # img_decode = cv2.imread("/home/sunjiayuan/slam/dataset/mav0/cam0/data/1413394914755760384.png", cv2.IMREAD_GRAYSCALE)
    # h,w = img_decode.shape
    # im_tensor = frame2tensor(img_decode, device)
    # pred0 = superpoint({"image": im_tensor})
    # pred = {}
    # pred = {**pred, **{k+'0': v for k, v in pred0.items()}}
    
    # kpts = pred['keypoints0'][0].cpu().numpy()

    # gim = cv2.GaussianBlur(img_decode,(7,7),0)
    # canny = cv2.Canny(img_decode, 50, 200)
    
    
    # # cv2.imshow("show",canny)
    # # cv2.waitKey(0)
    # kernel = np.array(([1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1], \
    #     [1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1]), dtype="float32")
    # dst = cv2.filter2D(canny, -1, kernel=kernel)

    # # ax1 = pl.subplot(131)
    # pl.imshow(img_decode, cmap=pl.cm.gray)
    # pl.xticks(()); pl.yticks(())

    # pl.savefig("1.png")
    

    # x,y = kpts.T - 10
    # # pl.plot(x,y,'+',c=(0,1,0),ms=10, scalex=0, scaley=0)


    # edge_mask = np.zeros((h,w))
    # for x in range(h):
    #     for y in range(w):
    #         edge_mask[x,y] = dst[x,y]*0.001

    # print(edge_mask)

    # # pl.subplot(132)
    # pl.imshow(img_decode, cmap=pl.cm.gray)
    # pl.xticks(()); pl.yticks(())
    # c = edge_mask
    # pl.imshow(transparent(c, 0.5, vmin=0, **kw))
    # pl.savefig("2.png")
    
    # sp_mask = np.zeros((h,w))
    # for pt in kpts:
    #     cv2.circle(sp_mask,(pt[0],pt[1]),R,color=(0.6),thickness=-1)

    # # ax1 = pl.subplot(133)
    
    # pl.imshow(img_decode, cmap=pl.cm.gray)
    # pl.xticks(()); pl.yticks(())
    # rela = sp_mask
    # pl.imshow(transparent(rela, 0.5, vmin=0, **kw))

    # # pl.gcf().set_size_inches(9, 2.73)
    # # pl.subplots_adjust(0.01,0.01,0.99,0.99,hspace=0.1)
    # pl.savefig("3.png")
    # # pdb.set_trace()