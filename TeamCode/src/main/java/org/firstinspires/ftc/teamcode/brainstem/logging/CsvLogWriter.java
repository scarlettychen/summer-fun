package org.firstinspires.ftc.teamcode.brainstem.logging;

import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

final class CsvLogWriter implements Closeable {

    private final BufferedWriter writer;
    private final File file;
    private final StringBuilder line = new StringBuilder(512);
    private boolean headerWritten;

    CsvLogWriter(File file, String header) throws IOException {
        this.file = file;
        File parent = file.getParentFile();
        if (parent != null) {

            parent.mkdirs();
        }
        this.writer = new BufferedWriter(new FileWriter(file, false), 16 * 1024);
        this.writer.write(header);
        this.writer.newLine();
        this.headerWritten = true;
    }

    File getFile() {
        return file;
    }

    void writeEntry(LogEntry entry) throws IOException {
        line.setLength(0);
        entry.appendCsv(line);
        writer.write(line.toString());
        writer.newLine();
    }

    void writeAll(List<LogEntry> batch) throws IOException {
        for (int i = 0; i < batch.size(); i++) {
            writeEntry(batch.get(i));
        }
    }

    void flush() throws IOException {
        writer.flush();
    }

    @Override
    public void close() throws IOException {
        writer.flush();
        writer.close();
    }

    boolean isHeaderWritten() {
        return headerWritten;
    }
}
