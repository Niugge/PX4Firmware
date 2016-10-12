package me.drton.jmavlib.log;

import java.util.List;

/**
 * User: ton Date: 03.06.13 Time: 16:18
 */
public class PX5LogMessage {
    public final PX5LogMessageDescription description;
    private final int multi_id;
    private final boolean is_active;
    private final List<Object> data;

    public PX5LogMessage(PX5LogMessageDescription description, int multi_id, boolean is_active, List<Object> data) {
        this.description = description;
        this.multi_id = multi_id;
        this.is_active = is_active;
        this.data = data;
    }

    public Object get(int idx) {
        return data.get(idx);
    }

    public long getLong(int idx) {
        return (Long) data.get(idx);
    }

    public Object get(String field) {
        Integer idx = description.fieldsMap.get(field);
        return idx == null ? null : data.get(idx);
    }

    public int get_id() {
        return multi_id;
    }

    public boolean is_active() {
        return is_active;
    }

    @Override
    public String toString() {
        return String.format("PX5LogMessage: type=%s, id=%s, name=%s, data=%s", description.type, multi_id, description.name, data);
    }
}
