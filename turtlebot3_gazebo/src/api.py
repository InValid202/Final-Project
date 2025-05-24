import xmlrpc.client
from datetime import datetime

# Get the current date
current_date = datetime.today().strftime('%Y-%m-%d')

url = 'https://natthachai1529-oee-test-odoo.odoo.com'
db = 'natthachai1529-oee-test-odoo-main-16715538'
username = 'admin'
password = 'admin'

common = xmlrpc.client.ServerProxy('{}/xmlrpc/2/common'.format(url))
common.version()
uid = common.authenticate(db, username, password, {})
# print(f'{uid}')
models = xmlrpc.client.ServerProxy('{}/xmlrpc/2/object'.format(url))

#partners = models.execute_kw(db, uid, password, 'res.partner', 'search', [[['is_company', '=', True]]])
#print(partners)

#models.execute_kw(db, uid, password, 'res.partner', 'search', [[['is_company', '=', True]]])
# สร้าง
#id = models.execute_kw(db, uid, password, 'my.oee', 'create', [{'date': current_date,'availability': "123"}])

#หาที่อยู่
#record_ids = models.execute_kw(db, uid, password, 'my.oee', 'search', [[['availability', '=', 123]]])
#print(f"Found record IDs: {record_ids}")

updated_data = {
    'availability': 90.0,
    'all_time': 1250,
    'run_time': 1100,
    'downtime': 150,
    'performance': 88.0,
    'idle_cycle_time': 30,
    'total_product2': 1600,
    'run_time2': 1050,
    'quality': 97.5,
    'good': 1560,
    'scrap': 40,
    'total_product': 1600
}
#เขียนทับ
result = models.execute_kw(db, uid, password, 'my.oee', 'write', [[20], updated_data])
