import csv
from urllib.parse import urlparse

def is_valid_url(url):
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except ValueError:
        return False

def generate_bom(csv_file, output_md):
    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        # Strip spaces from headers
        reader.fieldnames = [field.strip() for field in reader.fieldnames]
        
        items = []
        for row in reader:
            # Strip spaces from each value in the row
            item = {key: value.strip() for key, value in row.items()}
            items.append(item)
        
    total_cost = 0
    md_content = [
        "# Bill of Materials (BOM) for Digital Weight Project\n",
        "| Item Number | Part Name                       | Part Number | Quantity | Description                                    | Supplier         | Unit Price | Total Price | Purchase Link                          |",
        "|-------------|---------------------------------|-------------|----------|------------------------------------------------|------------------|------------|-------------|----------------------------------------|"
    ]
    
    for item in items:
        item_number = item['Item Number']
        part_name = item['Part Name']
        part_number = item['Part Number']
        quantity = item['Quantity']
        description = item['Description']
        supplier = item['Supplier']
        unit_price = item['Unit Price']
        purchase_link = item['Purchase Link']
        
        if not is_valid_url(purchase_link):
            purchase_link = ""
            buy_text = "No purchase link added"
        else:
            buy_text = "[Buy Here]"
        
        try:
            if unit_price == "N/A" or not unit_price:
                total_price = "N/A"
            else:
                unit_price_value = float(unit_price.replace('$', ''))
                total_price = unit_price_value * (float(quantity.split()[0]) if 'feet' in quantity else int(quantity))
                total_cost += total_price
                total_price = f"${total_price:.2f}"
                
            unit_price = f"${unit_price_value:.2f}" if unit_price != "N/A" else unit_price
        except ValueError:
            total_price = "N/A"
            unit_price = "N/A"
        
        md_content.append(
            f"| {item_number} | {part_name} | {part_number} | {quantity} | {description} | {supplier} | {unit_price} | {total_price} | {buy_text}({purchase_link}) |"
        )
    
    md_content.append(f"\n**Total Cost:** ${total_cost:.2f} (Approx)\n")
    md_content.append("\nNote: Prices are estimates and suppliers are placeholders. Actual costs and suppliers may vary.\n")
    
    with open(output_md, 'w') as mdfile:
        mdfile.write("\n".join(md_content))

if __name__ == "__main__":
    generate_bom('bom.csv', 'BOM.md')
