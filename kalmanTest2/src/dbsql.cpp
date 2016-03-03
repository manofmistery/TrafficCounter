/*
 * dbsql.cpp
 *
 *  Created on: Feb 17, 2016
 *      Author: ptspt
 */

#include <iostream>
#include <mysql++.h>
#include "dbsql.h"

using namespace mysqlpp;
using namespace std;

int dbinsert(int jmlMobil, int jmlTruksed, int jmlTrukbes, double kecepatan) {
	//double kec = 89.9012;
	String s("hello, world");
	cout << s << std::endl;
	try {
		Connection conn(false);
		//conn.connect("db_traffic", "localhost", "root", "ptspt2016");
		conn.connect("db_traffic", "localhost", "root", "ptspt2016");
		Query query = conn.query();
		/*query << "SELECT * FROM tb_kecepatan";
		StoreQueryResult ares = query.store();
		for (size_t i = 0; i < ares.num_rows(); i++)
			cout << "Id: " << ares[i]["id"] << " - Kecepatan: "
					<< ares[i]["kecepatan"] << endl;
		*/
		/* To insert stuff with escaping */
		query << "INSERT INTO tb_kendaraan " << "VALUES (" << "'', " /* This is left empty because the column is AUTO_INCREMENT */
		<<jmlMobil<<","
		<<jmlTruksed<<","
		<<jmlTrukbes<<","
		<<kecepatan<< ");";
		query.execute();
		/* That's it for INSERT */

	} catch (BadQuery er) { // handle any connection or
		// query errors that may come up
		cerr << "Error: " << er.what() << endl;
		return -1;
	} catch (const BadConversion& er) {
		// Handle bad conversions
		cerr << "Conversion error: " << er.what() << endl
				<< "\tretrieved data size: " << er.retrieved
				<< ", actual size: " << er.actual_size << endl;
		return -1;
	} catch (const Exception& er) {
		// Catch-all for any other MySQL++ exceptions
		cerr << "Error: " << er.what() << endl;
		return -1;
	}

	return (EXIT_SUCCESS);
}

