import numpy as np
import struct
import os


class PCDTools:
    def __init__(self, filename) -> None:
        self.__f = filename
        self.__header_str = None
        self.__fields =  None
        self.__size = None
        self.__num = None
        self.__type = None
        self.__fmt = None
        self.pts = None
        self.read_basic_info()
        self.get_fmt()
        self.load_pcd()

    def read_basic_info(self):
        header_str = ""
        with open(self.__f, "rb") as f:
            for i in range(11):
                header_str += f.readline().decode("utf-8")
        self.__header_str = header_str
        header_str_l = header_str.split("\n")

        l_3 = header_str_l[2].strip()
        self.__fields = l_3.split(" ")[1:]

        l_4 = header_str_l[3].strip()
        temp = l_4.split(" ")[1:]
        self.__size = [int(v) for v in temp]

        l_10 = header_str_l[9].strip()
        self.__num = int(l_10.split(" ")[-1])

        l_11 = header_str_l[10].strip()
        self.__type = l_11.split(" ")[-1]
        # print(f"{self.__fields}, {self.__size}, {self.__num}, {self.__type}")

    def load_pcd(self):
        if self.__type == "binary":
            print(self.__fmt)
            pts = []
            with open(self.__f, "rb") as f:
                for i in range(11):
                    f.readline()
                buf = f.read()
            n = sum(self.__size)
            for i in range(self.__num):
                p = struct.unpack(self.__fmt, buf[i * n: i * n + n])
                pts.append(p)
            self.pts = np.asarray(pts, dtype=np.float32)
        else:
            self.pts = np.loadtxt(self.__f, skiprows=11, dtype=np.float32)

    def ascii_to_binary(self):
        if self.__type == "binary":
            print("Alread Binary")
            return
        out = self.__f.split(".pcd")[0] + "_binary.pcd"
        h = self.__header_str.split("ascii")[0] + "binary\n"

        pts_b = bytes()
        for p in self.pts:
            for i, v in enumerate(self.__size):
                if v == 4:
                    pts_b += struct.pack("f", p[i])
                elif v == 1:
                    pts_b += struct.pack("B", int(p[i]))

        with open(out, "wb") as f:
            f.write(h.encode())
            f.write(pts_b)
        
    def binary_to_ascii(self):
        if self.__type == "ascii":
            print("Alread ASCII")
            return
        out = self.__f.split(".pcd")[0] + "_ascii.pcd"
        h = self.__header_str.split("binary")[0] + "ascii"
        np.savetxt(out, self.pts, fmt="%.3f", header=h, delimiter=" ", comments="")

    def rotation(self):
        # TODO
        pass

    def get_fmt(self):
        self.__fmt = ""
        for v in self.__size:
            if v == 4:
                self.__fmt += "f"
            elif v == 1:
                self.__fmt += "B"
            else:
                print("Unkown Type, In Function get_fmt")
                exit(-1)

    def __str__(self):
        return self.__header_str

if __name__ == "__main__":
    filename = "../data/0_ascii.pcd"
    tool = PCDTools(filename)
    tool.ascii_to_binary()