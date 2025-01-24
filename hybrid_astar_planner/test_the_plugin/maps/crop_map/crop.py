import cv2
import matplotlib.pyplot as plt

# 加载 PGM 图像
input_path = "room_mini.pgm"
output_path = "room_mini_cropped.pgm"
img = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)

# 定义裁剪区域 (y1:y2, x1:x2)
x1, y1, x2, y2 = 750, 780, 1150, 1150  # 示例坐标
cropped_img = img[y1:y2, x1:x2]

# 保存裁剪后的图像
cv2.imwrite(output_path, cropped_img)
print("裁剪完成并保存为:", output_path)

# 显示裁剪后的图像
plt.imshow(cropped_img, cmap='gray')
plt.title("Cropped Image")
plt.axis("off")  # 关闭坐标轴
plt.show()
