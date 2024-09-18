"""
生成二维码保存及对二维码解码输出
运行需要安装相应库
"""
import os
import qrcode
from PIL import Image
from pyzbar import pyzbar


def createQRCode1(content, save_path=None):
    """
    创建二维码图片，并保存
    :param content:二维码文本信息
    :param save_path:二维码保存地址
    :return:
    """

    qr = qrcode.QRCode(version=5, error_correction=qrcode.constants.ERROR_CORRECT_M, box_size=8, border=4)
    qr.add_data(data=content)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    if save_path:
        img.save(save_path)
    else:
        img.show()


def createQRCode2(content, icon_path, save_path=None):
    """
    创建带中心图片的二维码，并保存
    :param content: 二维码文本信息
    :param icon_path: 中心图片地址
    :param save_path: 二维码保存地址
    :return: 无返回值
    """

    # 判断中心图片是否存在
    if not os.path.exists(icon_path):
        raise FileExistsError(icon_path)

    # 创建二维码图片
    qr = qrcode.QRCode(version=5, error_correction=qrcode.constants.ERROR_CORRECT_H, box_size=8, border=4)
    qr.add_data(data=content)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white").convert('RGBA')

    # 调整二维码图片
    icon_img = Image.open(icon_path)
    code_width, code_height = img.size
    icon_img = icon_img.resize(
        (code_width // 4, code_height // 4), Image.ANTIALIAS)

    img.paste(icon_img, (code_width * 3 // 8, code_width * 3 // 8))

    if save_path:
        img.save(save_path)  # 保存二维码图片
        img.show()  # 显示二维码图片
    else:
        print("save error!")


def decode_qr_code(code_img_path):
    """
    识别二维码，对二维码进行编译，返回值
    :param code_img_path: 二维码的保存地址
    :return: 二维码的编译结果
    """
    if not os.path.exists(code_img_path):
        raise FileExistsError(code_img_path)

    return pyzbar.decode(Image.open(code_img_path), symbols=[pyzbar.ZBarSymbol.QRCODE])


if __name__ == "__main__":
    print("============QRcodetest===============")
    print("         1、Make a QRcode            ")
    print("         2、Scan a QRcode            ")
    print("=====================================")
    print("1、请输入二维码存储信息：")
    code_Data = input('>>:').strip()
    print("正在编码·······")

    # createQRCode2(code_Data, "img/QRcenter.jpg", "qrcode.png")  # 内容，center图片，生成二维码图片
    createQRCode1(code_Data, "qrcode.png")
    print("图片已保存，名称为：qrcode.png")
    results = decode_qr_code("qrcode.png")
    print("2、正在解码：")
    if len(results):
        print("解码结果是：")
        print(results[0].data.decode("utf-8"))
    else:
        print("无法识别")
