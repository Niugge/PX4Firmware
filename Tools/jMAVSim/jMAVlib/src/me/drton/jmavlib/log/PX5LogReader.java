package me.drton.jmavlib.log;

import java.io.EOFException;
import java.io.IOException;
import java.nio.BufferUnderflowException;
import java.util.HashMap;
import java.util.Map;

/**
 * User: ton Date: 03.06.13 Time: 14:18
 */
public class PX5LogReader extends BinaryLogReader {
    static final int HEADER_LEN = 3;
    private static final byte HEADER_MAGIC_NUM = (byte) 0x77;

    private long dataStart = 0;
    private Map<Integer, PX5LogMessageDescription> messageDescriptions
            = new HashMap<Integer, PX5LogMessageDescription>();
    private Map<String, String> fieldsList = null;
    private long time = 0;
    private PX5LogMessage lastMsg = null;
    private long sizeUpdates = -1;
    private long sizeMicroseconds = -1;
    private long startMicroseconds = -1;
    private long utcTimeReference = -1;
    private Map<String, Object> version = new HashMap<String, Object>();
    private Map<String, Object> parameters = new HashMap<String, Object>();

    public PX5LogReader(String fileName) throws IOException, FormatErrorException {
        super(fileName);
        readFormats();
        updateStatistics();
    }

    @Override
    public String getFormat() {
        return "PX5";
    }

    @Override
    public long getSizeUpdates() {
        return sizeUpdates;
    }

    @Override
    public long getStartMicroseconds() {
        return startMicroseconds;
    }

    @Override
    public long getSizeMicroseconds() {
        return sizeMicroseconds;
    }

    @Override
    public long getUTCTimeReferenceMicroseconds() {
        return utcTimeReference;
    }

    @Override
    public Map<String, Object> getVersion() {
        return version;
    }

    @Override
    public Map<String, Object> getParameters() {
        return parameters;
    }

    private void updateStatistics() throws IOException, FormatErrorException {
        position(dataStart);
        long packetsNum = 0;
        long timeStart = -1;
        long timeEnd = -1;
        while (true) {
            PX5LogMessage msg;
            try {
                msg = readMessage();
            } catch (EOFException e) {
                break;
            }
            // Time range
            if ("_TIMESTAMP".equals(msg.description.name)) {
                long t = msg.getLong(0);
                if (timeStart < 0) {
                    timeStart = t;
                }
                timeEnd = t;
            }
            packetsNum++;

            // Version
            if ("VER".equals(msg.description.name)) {
                String fw = (String) msg.get("FwGit");
                if (fw != null) {
                    version.put("FW", fw);
                }
                String hw = (String) msg.get("Arch");
                if (hw != null) {
                    version.put("HW", hw);
                }
            }

            // Parameters
            if ("PARM".equals(msg.description.name)) {
                parameters.put((String) msg.get("Name"), msg.get("Value"));
            }

            if ("GPS".equals(msg.description.name)) {
                if (utcTimeReference < 0) {
                    try {
                        int fix = ((Number) msg.get("Fix")).intValue();
                        long gpsT = ((Number) msg.get("GPSTime")).longValue();
                        if (fix >= 3 && gpsT > 0) {
                            utcTimeReference = gpsT - timeEnd;
                        }
                    } catch (Exception ignored) {
                    }
                }
            }
        }
        startMicroseconds = timeStart;
        sizeUpdates = packetsNum;
        sizeMicroseconds = timeEnd - timeStart;
        seek(0);
    }

    @Override
    public boolean seek(long seekTime) throws IOException, FormatErrorException {
        position(dataStart);
        lastMsg = null;
        if (seekTime == 0) {      // Seek to start of log
            time = 0;
            return true;
        }
        // Seek to specified timestamp without parsing all messages
        try {
            while (true) {
                int msgType = checkHeaderFillBuffer();
                PX5LogMessageDescription messageDescription = messageDescriptions.get(msgType);
                if (messageDescription == null) {
                    throw new RuntimeException("Unknown message type: " + msgType);
                }
                if (buffer.remaining() < messageDescription.fullLength) {
                    buffer.reset();
                    fillBuffer();
                    continue;
                }
                if ("_TIMESTAMP".equals(messageDescription.name)) {
                    PX5LogMessage msg = messageDescription.parseMessage(buffer);
                    long t = msg.getLong(0);
                    if (t > seekTime) {
                        // Time found
                        time = t;
                        buffer.reset();
                        return true;
                    }
                } else {
                    // Skip the message
                    buffer.position(buffer.position() + messageDescription.fullLength);
                }
            }
        } catch (EOFException e) {
            return false;
        }
    }

    private void applyMsg(Map<String, Object> update, PX5LogMessage msg) {
        if (msg.description.is_multi) {
            applyMsgAsName(update, msg, msg.description.name + "[" + msg.get_id() + "]");
        }
        if (msg.is_active()) {
            applyMsgAsName(update, msg, msg.description.name);
        }
    }

    void applyMsgAsName(Map<String, Object> update, PX5LogMessage msg, String msg_name) {
        PX5LogMessageDescription.FieldDescription[] fields = msg.description.fields;
        for (int i = 0; i < fields.length; i++) {
            PX5LogMessageDescription.FieldDescription field = fields[i];
            if (field.size >= 0) {
                for (int j = 0; j < field.size; j++) {
                    update.put(msg_name + "." + field.name + "[" + j + "]", ((Object[])msg.get(i))[j]);
                }
            } else {
                update.put(msg_name + "." + field.name, msg.get(i));
            }
        }

    }

    @Override
    public long readUpdate(Map<String, Object> update) throws IOException, FormatErrorException {
        long t = time;
        if (lastMsg != null) {
            applyMsg(update, lastMsg);
            lastMsg = null;
        }
        while (true) {
            PX5LogMessage msg = readMessage();

            if ("_TIMESTAMP".equals(msg.description.name)) {
                time = msg.getLong(0);
                if (t == 0) {
                    // The first TIME message
                    t = time;
                    continue;
                }
                break;
            }

            applyMsg(update, msg);
        }
        return t;
    }

    @Override
    public Map<String, String> getFields() {
        return fieldsList;
    }

    private void readFormats() throws IOException, FormatErrorException {
        fieldsList = new HashMap<String, String>();
        try {
            while (true) {
                if (fillBuffer() < 0) {
                    break;
                }
                while (true) {
                    if (buffer.remaining() < HEADER_LEN + 3) {  // Min size of format message is 3
                        break;
                    }
                    int msgType = checkHeader();     // Don't try to handle errors in formats
                    if (msgType == PX5LogMessageDescription.FORMAT.type) {
                        // Message description
                        PX5LogMessageDescription msgDescr = new PX5LogMessageDescription(buffer);
                        messageDescriptions.put(msgDescr.type, msgDescr);
                        if (msgDescr.name.charAt(0) != '_') {
                            for (int i = 0; i < msgDescr.fields.length; i++) {
                                PX5LogMessageDescription.FieldDescription fieldDescr = msgDescr.fields[i];
                                if (fieldDescr.size >= 0) {
                                    for (int j = 0; j < fieldDescr.size; j++) {
                                        fieldsList.put(msgDescr.name + "." + fieldDescr.name + "[" + j + "]", fieldDescr.type);
                                    }
                                } else {
                                    fieldsList.put(msgDescr.name + "." + fieldDescr.name, fieldDescr.type);
                                }
                            }
                        }
                    } else {
                        // Data message
                        dataStart = position();
                        return;
                    }
                }
            }
        } catch (EOFException ignored) {
        }
    }

    private int checkHeader() throws IOException, FormatErrorException {
        buffer.mark();
        if (buffer.get() != HEADER_MAGIC_NUM) {
            buffer.reset();
            throw new FormatErrorException(String.format("Invalid header at %s (0x%X)", position(), position()));
        }
        int msg_type = buffer.get() & 0xFF;
        buffer.reset();
        return msg_type;
    }

    private int checkHeaderFillBuffer() throws IOException, FormatErrorException {
        while (true) {
            if (buffer.remaining() < HEADER_LEN) {
                if (fillBuffer() == 0) {
                    throw new BufferUnderflowException();
                }
                continue;
            }
            return checkHeader();
        }
    }

    /**
     * Read next message from log
     *
     * @return log message
     * @throws IOException  on IO error
     * @throws EOFException on end of stream
     */
    public PX5LogMessage readMessage() throws IOException, FormatErrorException {
        int msgType = checkHeaderFillBuffer();
        PX5LogMessageDescription messageDescription = messageDescriptions.get(msgType);
        if (messageDescription == null) {
            throw new FormatErrorException("Unknown message type: " + msgType);
        }
        if (buffer.remaining() < messageDescription.fullLength) {
            fillBuffer();
            if (buffer.remaining() < messageDescription.fullLength) {
                throw new FormatErrorException("Unexpected end of file");
            }
        }
        return messageDescription.parseMessage(buffer);
    }

    public static void main(String[] args) throws Exception {
        PX5LogReader reader = new PX5LogReader("test.p5l");
        long tStart = System.currentTimeMillis();
        while (true) {
            try {
                PX5LogMessage msg = reader.readMessage();
            } catch (EOFException e) {
                break;
            }
        }
        long tEnd = System.currentTimeMillis();
        System.out.println(tEnd - tStart);
        reader.close();
    }
}
