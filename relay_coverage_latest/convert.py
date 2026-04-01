from PIL import Image

# List of image filenames (ordered)
image_files = [f"Coverage_{i}km.png" for i in range(1, 11)]

# Load images
frames = [Image.open(img) for img in image_files]

# Save as GIF
frames[0].save(
    "Coverage_1km_to_10km.gif",
    format="GIF",
    append_images=frames[1:],
    save_all=True,
    duration=800,   # time per frame in milliseconds
    loop=0          # 0 = infinite loop
)

print("✅ GIF created: Coverage_1km_to_10km.gif")