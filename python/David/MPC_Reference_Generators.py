import numpy as np
from PIL import Image, ImageDraw
from elliptic2 import *
import cv2
from matplotlib import cm

class ReferenceGenerator:

    def __init__(self, origin, alfa, semis=[1, 1], imagesize=[1000, 1000], scale=1):
        self.origin = origin
        self.alfa = alfa
        alfa = alfa * np.pi / 180
        self.rot = np.array([[np.cos(alfa), np.sin(alfa)], [-np.sin(alfa), np.cos(alfa)]])
        self.semis = semis
        self.M = np.diag(semis) @ self.rot
        self.M_inv = np.linalg.inv(self.M)
        self.originimage = [imagesize[0]/2 * scale, imagesize[1]/2 * scale]
        self.scale = scale
        self.imagesize = np.multiply(imagesize, scale)

    def getReferences(self, positions, normals):
        return np.zeros(len(positions))

    def getShape(self):
        p = np.add(self.originimage, self.origin)
        return [(int(p[0] - self.semis[0] * self.scale), int(p[1] - self.semis[1] * self.scale)),
                (int(p[0] + self.semis[0] * self.scale), int(p[1] + self.semis[1] * self.scale))]

    def getImage(self):
        cols = (np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3)) * 200).astype(np.uint8)
        img = Image.fromarray(cols)
        drw = ImageDraw.Draw(img)
        drw.ellipse(self.getShape(), outline='#777', width=3)
        img = img.rotate(self.alfa, fillcolor=(200, 200, 200))

        return img

    def updateRef(self, alfa, semis):
        self.alfa = alfa
        alfa = alfa * np.pi / 180
        self.rot = np.array([[np.cos(alfa), np.sin(alfa)], [-np.sin(alfa), np.cos(alfa)]])
        self.semis = semis
        self.M = np.diag(semis) @ self.rot
        self.M_inv = np.linalg.inv(self.M)


class EllipseRef(ReferenceGenerator):



    def getReferences(self, positions, normals):
        p = np.reshape(positions, (int(len(positions)/2), 2))
        n = np.reshape(normals, (int(len(normals)/2), 2))
        n = (self.rot @ n.T).T

        p = np.subtract(p, np.divide(self.originimage, self.scale))
        p = np.subtract(p, self.origin)
        p = (self.rot @ p.T).T

        refs = []
        for p_i, n_i in zip(p, n):

            # Find points on ellipse
            if abs(n_i[0]) > abs(n_i[1]):
                ye = 1 / np.sqrt(n_i[1]**2 / n_i[0]**2 / self.semis[0]**2 + 1 / self.semis[1]**2)
                xe = n_i[1] / n_i[0] * ye
            else:
                xe = 1 / np.sqrt(n_i[0] ** 2 / n_i[1] ** 2 / self.semis[1] ** 2 + 1 / self.semis[0] ** 2)
                ye = n_i[0] / n_i[1] * xe

            c = n_i[0] * xe + n_i[1] * ye
            c2 = n_i[0] * p_i[1] - n_i[1] * p_i[0]

            mat = np.linalg.inv(np.array([[n_i[0], n_i[1]], [-n_i[1], n_i[0]]]))
            x_ref = mat @ np.array([c, c2])

            x_ref = (self.rot.T @ x_ref) + np.divide(self.originimage, self.scale) + self.origin

            refs.append(x_ref)

        refs = np.array(refs)
        refs = refs.flatten()
        return refs


class EllipseRefClosestPoint(ReferenceGenerator):


    def getReferences(self, positions, normals):
        p = np.reshape(positions, (int(len(positions)/2), 2))
        n = np.reshape(normals, (int(len(normals)/2), 2))
        n = (self.rot @ n.T).T

        p = np.subtract(p, np.divide(self.originimage, self.scale))
        p = np.subtract(p, self.origin)
        p = (self.rot @ p.T).T

        # Now p is in the correct orientation

        refs = []
        for p_i, n_i in zip(p, n):

            alfa = np.arctan2(p_i[1], p_i[0])

            x_ref = np.array([self.semis[0] * np.cos(alfa), self.semis[1] * np.sin(alfa)])

            x_ref = (self.rot.T @ x_ref) + np.divide(self.originimage, self.scale) + self.origin

            refs.append(x_ref)

        refs = np.array(refs)
        refs = refs.flatten()
        return refs


class ImageRefClosestPoint(ReferenceGenerator):
    approach = False

    def generateRef(self, imageFile, order=20):
        # Open file
        im = cv2.imread(imageFile)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        # Generate points
        p, hierarchy = cv2.findContours(
            im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        idx = np.argmax([pi.shape[0] for pi in p])
        p = np.reshape(p[idx], (p[idx].shape[0], 2))

        p = np.subtract(p, np.average(p, axis=0))
        m = np.max(np.sqrt(np.sum(p**2, axis=1)))
        p = p / m / 2 + self.origin
        p = (self.M @ p.T).T

        coefs = elliptic_fourier_descriptors(p, order, normalize=False)
        x0, y0 = calculate_dc_coefficients(p)

        points = reconstruct_contour(coefs, locus=(x0,y0), num_points=300)

        self.points = points
        self.coefs = coefs
        self.x0 = x0
        self.y0 = y0

    def getReferences(self, positions, normals):
        p = np.reshape(positions, (int(len(positions) / 2), 2))
        n = np.reshape(normals, (int(len(normals) / 2), 2))
        n = (self.rot @ n.T).T

        p = np.subtract(p, np.divide(self.originimage, self.scale))
        # p = np.subtract(p, self.origin)
        R = - np.array([[-1, 0], [0, 1]])
        p = (R @ p.T).T

        # Now p is in the correct orientation
        mid1 = np.average(p, axis=0)
        mid2 = np.average(self.points, axis=0)

        refs = []
        for i, (p_i, n_i) in enumerate(zip(p, n)):

            diff = (self.M @ self.points.T).T - p_i
            diff = np.sum(diff**2, axis=1)
            diff = np.sqrt(diff)

            if self.approach:
                idx = (400-int(i / p.shape[0] * 300)) % 300#np.argmin(diff)
            else:
                idx = np.argmin(diff)

            x_ref = R @ self.M @self.points[idx, :] + np.divide(self.originimage, self.scale)

            refs.append(x_ref)

        refs = np.array(refs)
        refs = refs.flatten()
        return refs

    def getImage(self):
        cols = (np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3)) * 200).astype(np.uint8)
        img = Image.fromarray(cols)
        drw = ImageDraw.Draw(img)
        drw.ellipse(self.getShape(), outline='#777', width=3)
        img = img.rotate(self.alfa, fillcolor=(200, 200, 200))

        img = np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3), np.uint8) * 200
        points = (self.M @ self.points.T).T * self.scale + self.originimage

        points = points.reshape(-1, 1, 2)
        points = points.astype(int)
        img = cv2.polylines(img, [points], True, (0, 0, 0), thickness=3)

        return Image.fromarray(img)


class ImageCurvature(ReferenceGenerator):

    def generateRef(self, imageFile, numPoints, order=20):
        # Open file
        im = cv2.imread(imageFile)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        # Generate points
        p, hierarchy = cv2.findContours(
            im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        p = np.reshape(p[1], (p[1].shape[0], 2))

        p = np.subtract(p, np.average(p, axis=0))
        m = np.max(np.sqrt(np.sum(p ** 2, axis=1)))
        p = p / m / 2 + self.origin
        p = (self.M @ p.T).T

        self.order = order
        coefs = elliptic_fourier_descriptors(p, order, normalize=True)
        x0, y0 = calculate_dc_coefficients(p)

        points = reconstruct_contour(coefs, locus=(x0, y0), num_points=numPoints)

        self.calculateCurvature(points)

        self.cmap = cm.get_cmap('jet')

        self.cols = self.cmap(self.curvatures)

        self.points = points
        self.coefs = coefs
        self.x0 = x0
        self.y0 = y0

    def updateRef(self, alfa, semis):
        self.alfa = alfa
        alfa = alfa * np.pi / 180
        self.rot = np.array([[np.cos(alfa), np.sin(alfa)], [-np.sin(alfa), np.cos(alfa)]])
        self.semis = semis
        self.M = np.diag(semis) @ self.rot
        self.M_inv = np.linalg.inv(self.M)

    def getReferences(self, positions, normals):
        p = np.reshape(positions, (int(len(positions) / 2), 2))
        n = np.reshape(normals, (int(len(normals) / 2), 2))
        n = (self.rot @ n.T).T

        p = np.subtract(p, np.divide(self.originimage, self.scale))
        # p = np.subtract(p, self.origin)
        R = - np.array([[-1, 0], [0, 1]])
        p = (R @ p.T).T

        # Now p is in the correct orientation
        mid1 = np.average(p, axis=0)
        mid2 = np.average(self.points, axis=0)

        refs = []
        for i, (p_i, n_i) in enumerate(zip(p, n)):
            diff = (self.M @ self.points.T).T - p_i
            diff = np.sum(diff ** 2, axis=1)
            diff = np.sqrt(diff)

            idx = np.argmin(diff)  # (400-int(i / p.shape[0] * 300)) % 300#np.argmin(diff)

            x_ref = R @ self.M @ self.points[idx, :] + np.divide(self.originimage, self.scale)

            refs.append(x_ref)

        refs = np.array(refs)
        refs = refs.flatten()

        return refs

    def getReferencesC(self, num):
        return reconstruct_contour(self.coefs, num_points=num)

    def calculateCurvature(self, p):
        crvt1 = np.gradient(p, axis=0)
        crvt2 = np.gradient(crvt1, axis=0)

        x_d = crvt1[:, 0]
        y_d = crvt1[:, 1]
        x_dd = crvt2[:, 0]
        y_dd = crvt2[:, 1]

        k = (x_d * y_dd - y_d * x_dd) / np.power(x_d ** 2 + y_d ** 2, 3 / 2)

        k = (k - np.min(k)) / (np.max(k) - np.min(k))

        self.curvatures = k

    def getImage(self):
        cols = (np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3)) * 200).astype(np.uint8)
        img = Image.fromarray(cols)
        drw = ImageDraw.Draw(img)
        drw.ellipse(self.getShape(), outline='#777', width=3)
        img = img.rotate(self.alfa, fillcolor=(200, 200, 200))

        img = np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3), np.uint8) * 200
        points = (np.array([[1, 0], [0, -1]]) @ self.M @ self.points.T).T * self.scale + self.originimage

        points = points.reshape(-1, 1, 2)
        points = points.astype(int)

        for i, c in enumerate(self.cols):
            p1 = tuple(points[i, 0, :])
            p2 = tuple(points[(i+1) % points.shape[0], 0, :])

            if i == 0 or i == len(self.cols) - 1:
                img = cv2.line(img, p1, p2, (255, 255, 255), 10)
            else:
                img = cv2.line(img, p1, p2, c * 255, 10)



        return Image.fromarray(img)


class ImageFourierReal(ReferenceGenerator):

    def generateRef(self, imageFile, numPoints, order=20):

        # Open file
        im = cv2.imread(imageFile)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        im = cv2.flip(im, 0)

        # Generate points
        p, hierarchy = cv2.findContours(
            im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        p = np.reshape(p[1], (p[1].shape[0], 2))

        p = np.subtract(p, np.average(p, axis=0))
        m = np.max(np.sqrt(np.sum(p ** 2, axis=1)))
        p = p / m / 2 + self.origin
        p = (self.M @ p.T).T

        # Store information about order and save fourier series for contour
        self.order = order

        dxy = np.diff(np.vstack((p, p[0, :])), axis=0)
        dt = np.sqrt(np.sum(dxy**2, axis=1))
        t = np.concatenate(([0], np.cumsum(dt)))[:-1]
        T = t[-1]
        self.cnx = nurfft(p[:, 0], self.order, dt[:-1], t)
        self.cny = nurfft(p[:, 1], self.order, dt[:-1], t)

        points = np.real(np.vstack((irfft_n(self.cnx, numPoints), irfft_n(self.cny, numPoints))).T)

        self.calculateCurvature(points)

        self.cmap = cm.get_cmap('jet')

        self.cols = self.cmap(self.curvatures)

        self.points = points

    def updateRef(self, alfa, semis):
        self.alfa = alfa
        alfa = alfa * np.pi / 180
        self.rot = np.array([[np.cos(alfa), np.sin(alfa)], [-np.sin(alfa), np.cos(alfa)]])
        self.semis = semis
        self.M = np.diag(semis) @ self.rot
        self.M_inv = np.linalg.inv(self.M)

    def getReferences(self, positions, normals):
        p = np.reshape(positions, (int(len(positions) / 2), 2))
        n = np.reshape(normals, (int(len(normals) / 2), 2))
        n = (self.rot @ n.T).T

        p = np.subtract(p, np.divide(self.originimage, self.scale))
        # p = np.subtract(p, self.origin)
        R = - np.array([[-1, 0], [0, 1]])
        p = (R @ p.T).T

        # Now p is in the correct orientation
        mid1 = np.average(p, axis=0)
        mid2 = np.average(self.points, axis=0)

        refs = []
        for i, (p_i, n_i) in enumerate(zip(p, n)):
            diff = (self.M @ self.points.T).T - p_i
            diff = np.sum(diff ** 2, axis=1)
            diff = np.sqrt(diff)

            idx = np.argmin(diff)  # (400-int(i / p.shape[0] * 300)) % 300#np.argmin(diff)

            x_ref = R @ self.M @ self.points[idx, :] + np.divide(self.originimage, self.scale)

            refs.append(x_ref)

        refs = np.array(refs)
        refs = refs.flatten()

        return refs

    def getReferencesC(self, num):
        return reconstruct_contour(self.coefs, num_points=num)

    def calculateCurvature(self, p):
        crvt1 = np.gradient(p, axis=0)
        crvt2 = np.gradient(crvt1, axis=0)

        x_d = crvt1[:, 0]
        y_d = crvt1[:, 1]
        x_dd = crvt2[:, 0]
        y_dd = crvt2[:, 1]

        k = (x_d * y_dd - y_d * x_dd) / np.power(x_d ** 2 + y_d ** 2, 3 / 2)

        k = (k - np.min(k)) / (np.max(k) - np.min(k))

        self.curvatures = k

    def getImage(self):
        cols = (np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3)) * 200).astype(np.uint8)
        img = Image.fromarray(cols)
        drw = ImageDraw.Draw(img)
        drw.ellipse(self.getShape(), outline='#777', width=3)
        img = img.rotate(self.alfa, fillcolor=(200, 200, 200))

        img = np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3), np.uint8) * 200
        points = (np.array([[1, 0], [0, -1]]) @ self.points.T).T * self.scale + self.originimage

        points = points.reshape(-1, 1, 2)
        points = points.astype(int)

        for i, c in enumerate(self.cols):
            p1 = tuple(points[i, 0, :])
            p2 = tuple(points[(i+1) % points.shape[0], 0, :])

            if i == 0 or i == len(self.cols) - 1:
                img = cv2.line(img, p1, p2, (255, 255, 255), 3)
            else:
                img = cv2.line(img, p1, p2, 0*255, 3)



        return Image.fromarray(img)

class ImageFourierComplex(ReferenceGenerator):

    def generateRef(self, imageFile, numPoints, order=20):

        # Open file
        im = cv2.imread(imageFile)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        im = cv2.flip(im, 0)

        # Generate points
        p, hierarchy = cv2.findContours(
            im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        p = np.reshape(p[1], (p[1].shape[0], 2))

        p = np.subtract(p, np.average(p, axis=0))
        m = np.max(np.sqrt(np.sum(p ** 2, axis=1)))
        p = p / m / 2 + self.origin
        p = (self.M @ p.T).T

        # Store information about order and save fourier series for contour
        self.order = order

        dxy = np.diff(np.vstack((p, p[0, :])), axis=0)
        dt = np.sqrt(np.sum(dxy**2, axis=1))
        t = np.concatenate(([0], np.cumsum(dt)))[:-1]
        T = t[-1]
        pc = p[:, 0] + 1j * p[:, 1]
        self.cn = nufft(pc, self.order, dt[:-1], t)

        tmp = ifft_n(self.cn, numPoints)
        points = np.real(np.vstack((np.real(tmp), np.imag(tmp))).T)

        self.calculateCurvature(points)

        self.cmap = cm.get_cmap('jet')

        self.cols = self.cmap(self.curvatures)

        self.points = points

    def updateRef(self, alfa, semis):
        self.alfa = alfa
        alfa = alfa * np.pi / 180
        self.rot = np.array([[np.cos(alfa), np.sin(alfa)], [-np.sin(alfa), np.cos(alfa)]])
        self.semis = semis
        self.M = np.diag(semis) @ self.rot
        self.M_inv = np.linalg.inv(self.M)

    def getReferences(self, positions, normals):
        p = np.reshape(positions, (int(len(positions) / 2), 2))
        n = np.reshape(normals, (int(len(normals) / 2), 2))
        n = (self.rot @ n.T).T

        p = np.subtract(p, np.divide(self.originimage, self.scale))
        # p = np.subtract(p, self.origin)
        R = - np.array([[-1, 0], [0, 1]])
        p = (R @ p.T).T

        # Now p is in the correct orientation
        mid1 = np.average(p, axis=0)
        mid2 = np.average(self.points, axis=0)

        refs = []
        for i, (p_i, n_i) in enumerate(zip(p, n)):
            diff = (self.M @ self.points.T).T - p_i
            diff = np.sum(diff ** 2, axis=1)
            diff = np.sqrt(diff)

            idx = np.argmin(diff)  # (400-int(i / p.shape[0] * 300)) % 300#np.argmin(diff)

            x_ref = R @ self.M @ self.points[idx, :] + np.divide(self.originimage, self.scale)

            refs.append(x_ref)

        refs = np.array(refs)
        refs = refs.flatten()

        return refs

    def getReferencesC(self, num):
        return reconstruct_contour(self.coefs, num_points=num)

    def calculateCurvature(self, p):
        crvt1 = np.gradient(p, axis=0)
        crvt2 = np.gradient(crvt1, axis=0)

        x_d = crvt1[:, 0]
        y_d = crvt1[:, 1]
        x_dd = crvt2[:, 0]
        y_dd = crvt2[:, 1]

        k = (x_d * y_dd - y_d * x_dd) / np.power(x_d ** 2 + y_d ** 2, 3 / 2)

        k = (k - np.min(k)) / (np.max(k) - np.min(k))

        self.curvatures = k

    def getImage(self):
        cols = (np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3)) * 200).astype(np.uint8)
        img = Image.fromarray(cols)
        drw = ImageDraw.Draw(img)
        drw.ellipse(self.getShape(), outline='#777', width=3)
        img = img.rotate(self.alfa, fillcolor=(200, 200, 200))

        img = np.ones((int(self.imagesize[1]), int(self.imagesize[0]), 3), np.uint8) * 200
        points = (np.array([[1, 0], [0, -1]]) @ self.points.T).T * self.scale + self.originimage

        points = points.reshape(-1, 1, 2)
        points = points.astype(int)

        for i, c in enumerate(self.cols):
            p1 = tuple(points[i, 0, :])
            p2 = tuple(points[(i+1) % points.shape[0], 0, :])

            if i == 0 or i == len(self.cols) - 1:
                img = cv2.line(img, p1, p2, (255, 255, 255), 3)
            else:
                img = cv2.line(img, p1, p2, 0*255, 3)



        return Image.fromarray(img)
