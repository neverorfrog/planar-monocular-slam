import os
import glob
from PIL import Image
import re

def create_gif_from_images(image_pattern, output_gif, duration=200):
    """
    Create a GIF from a series of PNG images.
    
    Args:
        image_pattern: Pattern to match image files (e.g., "landmarks_*.png")
        output_gif: Output GIF filename
        duration: Duration between frames in milliseconds
    """
    # Get all matching files and sort them numerically
    image_files = glob.glob(image_pattern)
    
    # Sort by the number in the filename
    def extract_number(filename):
        match = re.search(r'_(\d+)\.png', filename)
        return int(match.group(1)) if match else 0
    
    image_files.sort(key=extract_number)
    
    if not image_files:
        print(f"No images found matching pattern: {image_pattern}")
        return
    
    # Load images
    images = []
    for file in image_files:
        img = Image.open(file)
        images.append(img)
    
    # Create GIF
    if images:
        images[0].save(
            output_gif,
            save_all=True,
            append_images=images[1:],
            duration=duration,
            loop=0  # 0 means infinite loop
        )
        print(f"Created GIF: {output_gif} with {len(images)} frames")

def main():
    # Get the figures directory
    figures_dir = "/home/neverorfrog/code/planar-monocular-slam/figures"
    
    # Create GIF for landmarks
    landmarks_pattern = os.path.join(figures_dir, "landmarks_*.png")
    landmarks_gif = os.path.join(figures_dir, "landmarks_animation.gif")
    create_gif_from_images(landmarks_pattern, landmarks_gif, duration=300)
    
    # Create GIF for trajectory
    trajectory_pattern = os.path.join(figures_dir, "trajectory_*.png")
    trajectory_gif = os.path.join(figures_dir, "trajectory_animation.gif")
    create_gif_from_images(trajectory_pattern, trajectory_gif, duration=300)