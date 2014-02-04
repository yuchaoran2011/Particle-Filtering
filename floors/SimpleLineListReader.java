package pf.floors;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.regex.Pattern;

import pf.utils.Line;

public class SimpleLineListReader {

	ArrayList<Line> mLines;
	
	public ArrayList<Line> readWalls(String filename) {
		
		//Log.d(TAG, "readWalls(filename = " + filename + ")");
		File inputFile = new File(filename); 
		if (!inputFile.isAbsolute()) {
			/*
			File root = Environment.getExternalStorageDirectory();
			inputFile = new File(root, filename);*/
		}
		
		mLines = new ArrayList<Line>();
		
		BufferedReader inputStream = null;
		try {
			inputStream = new BufferedReader(new FileReader(inputFile));
			
			Pattern pattern = Pattern.compile("[ ]");
			String line = null;
			while ((line = inputStream.readLine()) != null) {
				String[] coords = pattern.split(line);
				if (coords.length == 4) {
					double x1 = Double.valueOf(coords[0]);
					double y1 = Double.valueOf(coords[1]);
					double x2 = Double.valueOf(coords[2]);
					double y2 = Double.valueOf(coords[3]);
					mLines.add(new Line(x1, y1, x2, y2));
				}
			}
			
			inputStream.close();

		} catch (IOException ex) {
			ex.printStackTrace();
			
			try {
				if (inputStream != null)
					inputStream.close();
			} catch(IOException ex2) {
				ex2.printStackTrace();
			}
		}
		//Log.d(TAG, "SimpleLineListReader found " + mLines.size() + " lines");
		return mLines;
	}
}
