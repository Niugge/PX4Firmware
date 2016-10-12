package me.drton.jmavlib.log;

import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.util.*;

/**
 * User: ton Date: 03.06.13 Time: 14:35
 */
public class PX5LogMessageDescription {
    static PX5LogMessageDescription FORMAT = new PX5LogMessageDescription(0x00, 89, false, "_FORMAT", new FieldDescription[]{});

    static class FieldDescription {
        public final String name;
        public final String type;
        public final int size;

        public FieldDescription(String descr_str) {
            String[] p = descr_str.split(" ");
            name = p[1];
            if (p[0].contains("[")) {
                // Array
                String[] q = p[0].split("\\[");
                type = q[0];
                size = Integer.parseInt(q[1].split("\\]")[0]);
            } else {
                type = p[0];
                size = -1;
            }
        }

        public String getFullTypeString() {
            String size_str = (size >= 0) ? ("[" + size + "]") : "";
            return type + size_str;

        }

        public String toString() {
            return String.format("%s %s", getFullTypeString(), name);
        }
    }

    private static Charset charset = Charset.forName("latin1");

    public final int type;
    public final int fullLength;
    public final boolean is_multi;
    public final String name;
    public final FieldDescription[] fields;
    public final Map<String, Integer> fieldsMap = new HashMap<String, Integer>();

    private static String getString(ByteBuffer buffer, int len) {
        byte[] strBuf = new byte[len];
        buffer.get(strBuf);
        String[] p = new String(strBuf, charset).split("\0");
        return p.length > 0 ? p[0] : "";
    }

    public PX5LogMessageDescription(int type, int bodyLength, boolean is_multi, String name, FieldDescription[] fields) {
        this.type = type;
        this.fullLength = bodyLength + PX5LogReader.HEADER_LEN;
        this.is_multi = is_multi;
        this.name = name;
        this.fields = fields;
    }

    public PX5LogMessageDescription(ByteBuffer buffer) {
        buffer.get();   // Magic num
        buffer.get();   // Msg type
        buffer.get();   // Msg ID
        type = buffer.get() & 0xFF;
        fullLength = (buffer.get() & 0xFF) + PX5LogReader.HEADER_LEN;
        int format_len = buffer.get() & 0xFF;
        is_multi = (buffer.get() != 0);
        String[] descr_str = getString(buffer, format_len).split(":");
        name = descr_str[0];
        String[] fields_descrs_str = descr_str[1].split(";");
        fields = new FieldDescription[fields_descrs_str.length];
        for (int i = 0; i < fields_descrs_str.length; i++) {
            String field_format_str = fields_descrs_str[i];
            fields[i] = new FieldDescription(field_format_str);
            fieldsMap.put(fields[i].name, i);
        }
    }

    static Object getValue(ByteBuffer buffer, String type) {
        Object v;
        if (type.equals("float")) {
            v = buffer.getFloat();
        } else if (type.equals("double")) {
            v = buffer.getDouble();
        } else if (type.equals("int8_t") || type.equals("bool")) {
            v = (int) buffer.get();
        } else if (type.equals("uint8_t")) {
            v = buffer.get() & 0xFF;
        } else if (type.equals("int16_t")) {
            v = (int) buffer.getShort();
        } else if (type.equals("uint16_t")) {
            v = buffer.getShort() & 0xFFFF;
        } else if (type.equals("int32_t")) {
            v = buffer.getInt();
        } else if (type.equals("uint32_t")) {
            v = buffer.getInt() & 0xFFFFFFFFl;
        } else if (type.equals("int64_t")) {
            v = buffer.getLong();
        } else if (type.equals("uint64_t")) {
            v = buffer.getLong();
        } else if (type.equals("char")) {
            v = (char) buffer.get();
        } else {
            throw new RuntimeException("Unsupported type: " + type);
        }
        return v;
    }

    public PX5LogMessage parseMessage(ByteBuffer buffer) {
        buffer.get();   // Magic num, don't check
        buffer.get();   // Msg type, don't check
        int multi_id_raw = buffer.get();  // Msg Multi-ID
        List<Object> data = new ArrayList<Object>(fields.length);
        for (FieldDescription field : fields) {
            Object obj;
            if (field.size >= 0) {
                Object[] arr = new Object[field.size];
                for (int i = 0; i < field.size; i++) {
                    arr[i] = getValue(buffer, field.type);
                }
                obj = arr;
            } else {
                obj = getValue(buffer, field.type);
            }
            data.add(obj);
        }
        int multi_id = 0;
        boolean is_active = true;
        if (is_multi) {
            multi_id = (multi_id_raw & 0x7F);
            is_active = (multi_id_raw & 0x80) != 0;
        }
        return new PX5LogMessage(this, multi_id, is_active, data);
    }

    public List<String> getFields() {
        List<String> field_names = new ArrayList<String>(fields.length);
        for (FieldDescription field : fields) {
            field_names.add(field.name);
        }
        return field_names;
    }

    @Override
    public String toString() {
        return String.format("PX5LogMessageDescription: type=%s, fullLength=%s, name=%s, fields=%s", type,
                fullLength, name, Arrays.asList(fields));
    }
}
