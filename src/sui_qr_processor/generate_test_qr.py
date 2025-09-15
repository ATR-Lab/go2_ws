#!/usr/bin/env python3
"""
Generate test QR codes for sui_qr_processor testing.
This script creates QR codes with the expected JSON format for testing the node.
"""

import json
import qrcode
from PIL import Image
import argparse

def generate_test_qr(output_file="test_qr.png", network="testnet"):
    """Generate a test QR code with sample Sui transaction data."""
    
    # Sample transaction data (from the sui-py example)
    test_data = {
        "bytes": "AAAEAQBX81xJQM5DHo5/jceY0CRyy75ofrHiPR08Z87V+uJp0SUeUCIAAAAAIOG7Q2BqQ7ubDu+AMmcKnOMtQ9qlCPVyov5TAUwSBiU5AAgBAAAAAAAAAAEB+kXkr+JWG8JF5msZDy5DkcCptMOkz7UUC2RKVX4Q5Ox6LDkiAAAAAAEBAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAGAQAAAAAAAAAAAwIBAAABAQEAALkI7DF3LJccTHGo/QLJ4W/2TmTcck15IDO06bHDVJHrCXJvYm90X3BldAJoaQAAALkI7DF3LJccTHGo/QLJ4W/2TmTcck15IDO06bHDVJHrCXJvYm90X3BldApmZWVkX3RyZWF0AAQBAgADAAAAAAIBAAEDAM0yfOn6ogI/ApPwOB063148bFd7ZbYSZKWoCNyCeblkAU3b6gkObn2/Rr8HB9Vj68sMNC8xqn2QVUDx5HVQNrpUWWVQIgAAAAAg9Rr3yuGntheUmkysknxBWwks+6Wqbh41Z64mAPCD8c3NMnzp+qICPwKT8DgdOt9ePGxXe2W2EmSlqAjcgnm5ZOgDAAAAAAAAhNgxAAAAAAAA",
        "signature": "AAt4ih9jPcbdc3SkSiBI6gbL+3MRnnHs5V3hM1ptgHr/AEu/YjXx2QTh5/orJqYSji/qwvW/zWU0fZqJ8oSWywdunWqkb/0h4vSCCYj1w2OU84nFcZtk45+ZI+TTBcdtYg==",
        "chain": f"sui:{network}"
    }
    
    # Convert to JSON string
    json_data = json.dumps(test_data, separators=(',', ':'))
    
    print(f"Generating QR code for {network} network...")
    print(f"Data length: {len(json_data)} characters")
    print(f"JSON data preview: {json_data[:100]}...")
    
    # Create QR code
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    
    qr.add_data(json_data)
    qr.make(fit=True)
    
    # Create image
    img = qr.make_image(fill_color="black", back_color="white")
    
    # Save image
    img.save(output_file)
    print(f"✅ QR code saved to: {output_file}")
    
    return json_data

def generate_invalid_qr(output_file="invalid_qr.png"):
    """Generate an invalid QR code for testing error handling."""
    
    invalid_data = {
        "invalid": "data",
        "missing": "required_fields"
    }
    
    json_data = json.dumps(invalid_data)
    
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    
    qr.add_data(json_data)
    qr.make(fit=True)
    
    img = qr.make_image(fill_color="red", back_color="white")
    img.save(output_file)
    
    print(f"✅ Invalid QR code saved to: {output_file}")
    return json_data

def main():
    parser = argparse.ArgumentParser(description="Generate test QR codes for sui_qr_processor")
    parser.add_argument("--network", default="testnet", choices=["testnet", "mainnet", "devnet"],
                       help="Sui network for the test QR code")
    parser.add_argument("--output", default="test_qr.png",
                       help="Output file name for the QR code image")
    parser.add_argument("--invalid", action="store_true",
                       help="Generate invalid QR code for error testing")
    
    args = parser.parse_args()
    
    try:
        if args.invalid:
            generate_invalid_qr(args.output)
        else:
            generate_test_qr(args.output, args.network)
        
        print(f"\n📱 To test with the QR code:")
        print(f"   1. Display {args.output} on a screen")
        print(f"   2. Point your camera at the QR code")
        print(f"   3. Run: ros2 launch sui_qr_processor sui_qr_processor.launch.py")
        print(f"   4. Monitor: ros2 topic echo /sui_qr_processor_node/transaction_status")
        
    except ImportError:
        print("❌ Missing dependencies. Install with:")
        print("   pip install qrcode[pil]")
    except Exception as e:
        print(f"❌ Error generating QR code: {e}")

if __name__ == "__main__":
    main()
