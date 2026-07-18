from PIL import Image
import sys
import os

def rgb565(r, g, b):
    r = (r >> 3) & 0x1F
    g = (g >> 2) & 0x3F
    b = (b >> 3) & 0x1F
    return (r << 11) | (g << 5) | b

def main(png_path, output_path):
    # Load with alpha and rotate 90 degrees clockwise
    src = Image.open(png_path).convert('RGBA')
    img = src.transpose(Image.ROTATE_270)

    # Trim fully transparent borders to maximize visible content size
    alpha = img.split()[3]
    bbox = alpha.getbbox()
    if bbox:
        img = img.crop(bbox)

    # Target canvas: driver coordinate system (landscape) 536x240
    # The physical panel is 240x536; driver rotates writes internally.
    # display_width = 240
    # display_height = 536

    display_width = 250
    display_height = 222

    # Scale to fit within display keeping aspect ratio (no cropping)
    w, h = img.size
    scale = min(display_width / w, display_height / h)
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    img = img.resize((new_w, new_h), Image.LANCZOS)

    # Create black background and alpha-composite the rotated, scaled logo
    canvas = Image.new('RGBA', (display_width, display_height), (0, 0, 0, 255))
    x_off = (display_width - new_w) // 2
    y_off = (display_height - new_h) // 2
    # Center paste with alpha (no cropping expected since we fit to canvas)
    canvas.paste(img, (x_off, y_off), img)

    # Convert to RGB for 565 output. Transparent parts remain black.
    rgb = canvas.convert('RGB')
    pixels = list(rgb.getdata())

    with open(output_path, 'w') as f:
        f.write(f'#define LOGO_WIDTH {display_width}\n')
        f.write(f'#define LOGO_HEIGHT {display_height}\n')
        f.write('const uint16_t logo[] = {\n')
        for i in range(0, len(pixels), display_width):
            row = pixels[i:i+display_width]
            f.write('    ' + ', '.join(f'0x{rgb565(r,g,b):04X}' for r, g, b in row) + ',\n')
        f.write('};\n')

if __name__ == '__main__':
    # set default png path if not provided
    if len(sys.argv) < 2:
        print("No input arguments provided, using default logo.png in the script directory.")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        sys.argv.append(os.path.join(script_dir, 'logo.png'))
        # add default output path
        sys.argv.append(os.path.join(script_dir, 'logo.h'))

    if len(sys.argv) != 3:
        print("Usage: python convert_logo.py <png_path> <output_path>")
        sys.exit(1)     

    main(sys.argv[1], sys.argv[2])
