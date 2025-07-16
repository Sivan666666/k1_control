import mujoco
import os

# ==============================================================================
# --- 配置区 ---
# 由于脚本和URDF在同一个文件夹，我们让路径自动生成
# ==============================================================================

# 获取当前脚本所在的目录
# 这应该是 '/home/hwk/ros_ws/piper_ros/src/piper_description/urdf'
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))

# 输入文件名 (与脚本在同一目录下)
URDF_FILENAME = "k1_description.urdf"

# 输出文件名 (将保存在同一目录下)
MJCF_FILENAME = "k1_urdf.xml"

# 组合成完整路径
urdf_input_path = os.path.join(CURRENT_DIR, URDF_FILENAME)
mjcf_output_path = os.path.join(CURRENT_DIR, MJCF_FILENAME)

# ==============================================================================
# --- 转换代码 ---
# ==============================================================================

def convert_with_python(urdf_path, mjcf_path):
    """
    读取一个路径已经修正好的URDF文件，并将其转换为MJCF。
    这个方法比命令行工具更可靠。
    """
    print(f"Attempting to load URDF: {urdf_path}")
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

    # MuJoCo的Python加载器能更好地处理相对路径
    try:
        model = mujoco.MjModel.from_xml_path(urdf_path)
        print("✅ URDF model loaded successfully in Python.")
    except Exception as e:
        print(f"❌ FATAL ERROR: MuJoCo could not load the URDF file. Details: {e}")
        print("\nTroubleshooting:")
        print("1. Please double-check that the relative paths inside your URDF (e.g., '../meshes/') are correct.")
        print("2. Ensure you have read permissions for all mesh files.")
        return

    # 将加载好的模型保存为干净的MJCF XML文件
    try:
        mujoco.mj_saveLastXML(mjcf_path, model)
        print(f"\n✅ Successfully converted and saved new MJCF model to: {mjcf_path}")
        print("\nThis file is now ready to be used!")
    except Exception as e:
        print(f"❌ FATAL ERROR: Could not save the model to XML. Details: {e}")


if __name__ == '__main__':
    convert_with_python(urdf_input_path, mjcf_output_path)
